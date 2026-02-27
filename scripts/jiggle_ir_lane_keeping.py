from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml
from pathlib import Path
import time
import RPi.GPIO as GPIO


def main(config_path):
    
    config = load_yaml(config_path)

    print("Initializing motors...")
    control = Control(**config["control"])

    IR_PIN = 21
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    BLACK_STATE = 1

    FORWARD_SPEED = 1650   # <<< adjust speed here
    LEFT_TURN = 80
    RIGHT_TURN = 120
    TURN_DELAY = 0.05

    print("Starting tape following...")
    control.forward(FORWARD_SPEED)

    try:
        while True:

            state = GPIO.input(IR_PIN)

            if state == BLACK_STATE:
                control.turn(LEFT_TURN)
                time.sleep(TURN_DELAY)
                control.forward(FORWARD_SPEED)

            else:
                control.turn(RIGHT_TURN)
                time.sleep(TURN_DELAY)
                control.forward(FORWARD_SPEED)

    finally:
        GPIO.cleanup()
        print("Reseting motors to neutral...")
        control.shutdown()


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))