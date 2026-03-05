import RPi.GPIO as GPIO

class IR: 
    """Sensor interface for infrared (IR)"""

    def __init__(self, IR_PIN: int):
        self.IR_PIN = IR_PIN
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def __del__(self):
        """Cleans up the IR GPIO pin."""
        GPIO.cleanup(self.IR_PIN)

    def get_ir_state(self):
        return GPIO.input(self.IR_PIN)