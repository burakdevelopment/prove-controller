from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QSlider,
    QApplication, QGridLayout, QMessageBox, QGroupBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont

from core.mavlink_handler import MAVLinkHandler
from core.gamepad_handler import GamepadHandler

import threading
import math
import sys
import logging

logger = logging.getLogger("GUI")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(ch)


def axis_to_pwm(axis_val, pwm_min=1100, pwm_max=1900, deadzone=0.05):
    if abs(axis_val) < deadzone:
        axis_val = 0.0
    
    mid = (pwm_max + pwm_min) / 2.0
    span = (pwm_max - pwm_min) / 2.0
    pwm = int(mid + (axis_val * span))
    return max(pwm_min, min(pwm_max, pwm))

class Signals(QObject):
    heartbeat = pyqtSignal(int, int)
    motor_feedback = pyqtSignal(list)
    connection_changed = pyqtSignal(bool)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SubController v1.1 - Motor Test (Bay Burak)")
        self.setGeometry(120, 120, 820, 520)
        self.signals = Signals()

        
        self.mav = MAVLinkHandler()
        self.gamepad = GamepadHandler()

        
        self.connected = False
        self.channel_count = 6  
        self.sliders = []
        self.rpm_labels = []
        self.current_pwms = [1500]*self.channel_count

        self._setup_ui()
        self._connect_signals()

        
        self._tx_timer = QTimer()
        self._tx_timer.setInterval(100)  # ms
        self._tx_timer.timeout.connect(self._periodic_tx)
        self._tx_timer.start()

        
        self.gamepad.on_input = self._on_gamepad_input
        self.gamepad.start()

    def _setup_ui(self):
        main_layout = QVBoxLayout()
        title = QLabel("SubController v1.1 - Motor Test")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        main_layout.addWidget(title)

        
        conn_layout = QHBoxLayout()
        self.status_label = QLabel("Bağlantı: ❌")
        self.connect_btn = QPushButton("Pixhawk'a Bağlan (Otomatik Port Taraması)")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.disconnect_btn = QPushButton("Bağlantıyı Kes")
        self.disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_btn.setEnabled(False)
        conn_layout.addWidget(self.status_label)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        main_layout.addLayout(conn_layout)

        
        grid = QGridLayout()
        for i in range(self.channel_count):
            group = QGroupBox(f"Motor {i+1}")
            gl = QVBoxLayout()
            lbl = QLabel("PWM: 1500")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(1100, 1900)
            slider.setValue(1500)
            slider.valueChanged.connect(self._make_slider_callback(i))
            self.sliders.append(slider)
            rpm_lbl = QLabel("RPM: ---")
            rpm_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.rpm_labels.append(rpm_lbl)
            gl.addWidget(lbl)
            gl.addWidget(slider)
            gl.addWidget(rpm_lbl)
            group.setLayout(gl)
            
            slider._pwm_label = lbl
            grid.addWidget(group, i//3, i%3)
        main_layout.addLayout(grid)

        
        btn_layout = QHBoxLayout()
        self.all_stop_btn = QPushButton("ALL STOP (1100 PWM)")
        self.all_stop_btn.clicked.connect(self._on_all_stop)
        self.test_seq_btn = QPushButton("Test Sequence")
        self.test_seq_btn.clicked.connect(self._on_test_sequence)
        btn_layout.addWidget(self.all_stop_btn)
        btn_layout.addWidget(self.test_seq_btn)
        main_layout.addLayout(btn_layout)

        
        self.joystick_label = QLabel("Joystick: Bulunamadı")
        main_layout.addWidget(self.joystick_label)

        self.setLayout(main_layout)

    def _connect_signals(self):
        self.signals.heartbeat.connect(self._on_heartbeat)
        self.signals.motor_feedback.connect(self._on_motor_feedback)
        self.signals.connection_changed.connect(self._on_connection_changed)
        
        self.mav.on_heartbeat = lambda t,a: self.signals.heartbeat.emit(t,a)
        self.mav.on_motor_feedback = lambda arr: self.signals.motor_feedback.emit(arr)

    def _make_slider_callback(self, idx):
        def callback(val):
            self.current_pwms[idx] = val
            slider = self.sliders[idx]
            try:
                slider._pwm_label.setText(f"PWM: {val}")
            except Exception:
                pass
        return callback

    def _on_connect_clicked(self):
        self.connect_btn.setEnabled(False)
        self.status_label.setText("Bağlantı: Bağlanıyor...")
        
        t = threading.Thread(target=self._connect_background, daemon=True)
        t.start()

    def _connect_background(self):
        ok = self.mav.try_connect()
        self.signals.connection_changed.emit(ok)

    def _on_connection_changed(self, ok: bool):
        if ok:
            self.connected = True
            self.status_label.setText("Bağlantı: ✅ Pixhawk Bağlandı")
            self.disconnect_btn.setEnabled(True)
            self.connect_btn.setEnabled(False)
        else:
            self.connected = False
            self.status_label.setText("Bağlantı: ❌ Bağlantı başarısız")
            self.disconnect_btn.setEnabled(False)
            self.connect_btn.setEnabled(True)

    def _on_disconnect_clicked(self):
        self.mav.disconnect()
        self.connected = False
        self.status_label.setText("Bağlantı: ❌")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def _periodic_tx(self):
        if not self.connected:
            return
        
        vals = [0]*18
        for i in range(self.channel_count):
            vals[i] = int(self.current_pwms[i])
        self.mav.send_rc_overrides(vals)

    def _on_all_stop(self):
        reply = QMessageBox.question(self, "All Stop", "Tüm motorları 1100 PWM ile durdurmak istediğine emin misin? (Pervane takılı değilken test et.)",
                                     QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            self.mav.send_all_stop(stop_pwm=1100, channel_indices=list(range(self.channel_count)))
            
            for i, s in enumerate(self.sliders):
                s.setValue(1100)

    def _on_test_sequence(self):
        
        threading.Thread(target=self._run_test_sequence, daemon=True).start()

    def _run_test_sequence(self):
        if not self.connected:
            QMessageBox.information(self, "Bilgi", "Önce Pixhawk'a bağlan.")
            return
        base = 1300
        for i in range(self.channel_count):
            
            for j in range(self.channel_count):
                val = base if j != i else 1600
                self.sliders[j].setValue(val)
            time_sleep = 1.2
            threading.Event().wait(time_sleep)
        
        for j in range(self.channel_count):
            self.sliders[j].setValue(1500)

    def _on_gamepad_input(self, axes, buttons):
        if len(axes) == 0:
            self.joystick_label.setText("Joystick: Bulunamadı")
            return
        self.joystick_label.setText(f"Joystick: {len(axes)} axes, {len(buttons)} buttons")
        ax_fwd = -axes[1] if len(axes) > 1 else 0.0
        ax_strafe = axes[0] if len(axes) > 0 else 0.0
        ax_yaw = axes[2] if len(axes) > 2 else 0.0
        ax_vert = -axes[3] if len(axes) > 3 else 0.0  

        # example mixing to 6 motors (this mixing is generic; adapt to actual thruster geometry)
        # Basic mixing:
        # motor1 = fwd + strafe + yaw
        # motor2 = fwd - strafe - yaw
        # motor3 = fwd + strafe - yaw
        # motor4 = fwd - strafe + yaw
        # motor5 = vert (left)
        # motor6 = vert (right)

        def mix(a, b, c):
            return a + b + c

        motors = [0]*6
        motors[0] = mix(ax_fwd, ax_strafe, ax_yaw)
        motors[1] = mix(ax_fwd, -ax_strafe, -ax_yaw)
        motors[2] = mix(ax_fwd, ax_strafe, -ax_yaw)
        motors[3] = mix(ax_fwd, -ax_strafe, ax_yaw)
        
        motors[4] = ax_vert
        motors[5] = ax_vert

        
        for i in range(self.channel_count):
            pwm = axis_to_pwm(motors[i])
            
            self.sliders[i].setValue(pwm)

    def _on_heartbeat(self, comp_type, autopilot):
        
        self.status_label.setText(f"Bağlantı: ✅ Pixhawk (type={comp_type}, autopilot={autopilot})")

    def _on_motor_feedback(self, arr):
        
        for i in range(min(self.channel_count, len(arr))):
            try:
                
                self.rpm_labels[i].setText(f"RPM/PWM: {int(arr[i])}")
            except Exception:
                self.rpm_labels[i].setText("RPM/PWM: ---")

    def closeEvent(self, event):
        try:
            self.gamepad.stop()
        except Exception:
            pass
        try:
            self.mav.disconnect()
        except Exception:
            pass
        event.accept()
