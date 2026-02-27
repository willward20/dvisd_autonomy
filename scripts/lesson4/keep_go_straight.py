"""
Run command:
    python3 -m scripts.lesson4.keep_go_straight
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

    GPIO.setmode(GPIO.BCM)

    FORWARD_SPEED = 1637
    STOP_SPEED = 1620
    STRAIGHT = 100          # neutral steering
    DELAY = 1

    try:
        while True:
            control.turn(STRAIGHT)
            control.forward(FORWARD_SPEED)
            time.sleep(DELAY)
            control.stop()

    finally:
        GPIO.cleanup()
        print("Reseting motors to neutral...")
        control.shutdown()


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))