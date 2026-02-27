"""
Run command:
    python3 -m scripts.lesson4.jiggle_ir_lane_keeping
"""

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

    FORWARD_SPEED = 1637
    STOP_SPEED = 1620
    LEFT_TURN = 80
    RIGHT_TURN = 120
    STRAIGHT = 100          # neutral steering
    TURN_DELAY = 1

    print("Starting tape following...")
    control.forward(FORWARD_SPEED)

    try:
        while True:

            state = GPIO.input(IR_PIN)
            color = "BLACK" if state == BLACK_STATE else "WHITE"
            print("IR State:", color, state)

            if state == BLACK_STATE:
                control.turn(LEFT_TURN)
                control.forward(FORWARD_SPEED)
                print("left")
            else:
                control.turn(RIGHT_TURN)
                control.forward(FORWARD_SPEED)
                print("right")

            time.sleep(TURN_DELAY)
            control.stop()

            # control.forward(STOP_SPEED)
            # time.sleep(TURN_DELAY)
            # control.stop()

    finally:
        GPIO.cleanup()
        print("Reseting motors to neutral...")
        control.shutdown()


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))