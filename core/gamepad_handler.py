import threading
import time
import logging
from typing import Callable, Optional, List

import pygame

logger = logging.getLogger("Gamepad")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(ch)

class GamepadHandler:
    def __init__(self):
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.joystick = None
        
        self.on_input: Optional[Callable[[List[float], List[int]], None]] = None
        self.poll_interval = 0.02  

    def start(self):
        if self._running:
            return
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        logger.info(f"Detected {count} joystick(s)")
        if count == 0:
            logger.info("No joystick found.")
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logger.info(f"Using joystick: {self.joystick.get_name()}")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        try:
            if self.joystick:
                self.joystick.quit()
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass

    def _loop(self):
        while self._running:
            try:
                pygame.event.pump()
                if self.joystick:
                    axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
                    buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                    if self.on_input:
                        try:
                            self.on_input(axes, buttons)
                        except Exception:
                            pass
                else:
                    
                    pass
            except Exception as e:
                logger.info(f"Gamepad loop exception: {e}")
            time.sleep(self.poll_interval)
