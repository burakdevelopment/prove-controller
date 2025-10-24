import threading
import time
import logging
from typing import List, Optional, Callable

from pymavlink import mavutil
import serial.tools.list_ports

logger = logging.getLogger("MAVHandler")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(ch)

DEFAULT_BAUDS = [57600, 115200]

CHANNEL_COUNT = 18

class MAVLinkHandler:
    def __init__(self):
        self.master: Optional[mavutil.mavlink_connection] = None
        self.connected = False
        self._rx_thread = None
        self._stop_rx = threading.Event()
        self.on_heartbeat: Optional[Callable[[int,int], None]] = None
        self.on_motor_feedback: Optional[Callable[[List[int]], None]] = None

    def find_candidate_ports(self) -> List[str]:
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(p.device)
        
        if not ports:
            ports.extend(['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1'])
        return ports

    def try_connect(self, specific_port: Optional[str] = None, timeout=5) -> bool:
        ports = [specific_port] if specific_port else self.find_candidate_ports()
        for port in ports:
            for baud in DEFAULT_BAUDS:
                try:
                    logger.info(f"Trying {port} @ {baud}")
                    master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
                    
                    master.wait_heartbeat(timeout=timeout)
                    
                    self.master = master
                    self.connected = True
                    logger.info(f"Connected to MAVLink on {port} @ {baud}")
                    
                    self._stop_rx.clear()
                    self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                    self._rx_thread.start()
                    return True
                except Exception as e:
                    logger.info(f"Connection failed for {port} @ {baud}: {e}")
        return False

    def disconnect(self):
        self._stop_rx.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1)
        try:
            if self.master:
                self.master.close()
        except Exception:
            pass
        self.master = None
        self.connected = False
        logger.info("Disconnected from MAVLink")

    def _rx_loop(self):
        
        while not self._stop_rx.is_set() and self.master:
            try:
                msg = self.master.recv_match(timeout=1)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    if self.on_heartbeat:
                        try:
                            self.on_heartbeat(msg.type, msg.autopilot)
                        except Exception:
                            pass
                
                if msg_type == 'MOTOR_OUTPUTS' or msg_type == 'ESC_STATUS' or msg_type == 'MOTOR_OUTPUTS_RAW':
                    
                    try:
                        
                        motor_vals = []
                        if hasattr(msg, 'motor'):
                            motor_vals = list(msg.motor)
                        elif hasattr(msg, 'motor_output'):
                            motor_vals = list(msg.motor_output)
                        elif hasattr(msg, 'values'):
                            motor_vals = list(msg.values)
                        
                        motor_vals = (motor_vals + [0]*6)[:6]
                        if self.on_motor_feedback:
                            self.on_motor_feedback(motor_vals)
                    except Exception:
                        pass
                
            except Exception as e:
                logger.info(f"RX loop exception: {e}")
                time.sleep(0.5)

    def send_rc_overrides(self, pwm_values: List[int]):
        if not self.connected or not self.master:
            return False
        
        vals = list(pwm_values)[:CHANNEL_COUNT]
        vals += [0] * (CHANNEL_COUNT - len(vals))
        try:
            
            args = [self.master.target_system, self.master.target_component] + vals
            self.master.mav.rc_channels_override_send(*args)
            return True
        except Exception:
            
            try:
                self.master.mav.rc_channel_override_send(self.master.target_system, self.master.target_component, *vals)
                return True
            except Exception as e:
                logger.info(f"Failed to send rc override: {e}")
                return False

    def send_all_stop(self, stop_pwm=1100, channel_indices=None):
        if channel_indices is None:
            
            channel_indices = list(range(6))
        pwm = [0]*CHANNEL_COUNT
        for idx in channel_indices:
            if 0 <= idx < CHANNEL_COUNT:
                pwm[idx] = stop_pwm
        return self.send_rc_overrides(pwm)
