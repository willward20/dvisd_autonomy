import RPi.GPIO as GPIO
import time

class Sonar: 
    """Sensor interface for ultrasonic (sonar)"""

    def __init__(
        self, 
        TRIG: int, 
        ECHO: int
    ):
        """Initialize GPIO pins and set TRIG to low."""
        self.TRIG = TRIG
        self.ECHO = ECHO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.output(TRIG, False)
        time.sleep(0.2)

    def __del__(self):
        """Cleans up the sonar GPIO pins."""
        GPIO.cleanup(self.TRIG)
        GPIO.cleanup(self.ECHO)

    def send_pulse(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

    def get_sonar_signal(self):
        return GPIO.input(self.ECHO)