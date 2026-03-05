"""
Run command:
    python3 -m scripts.lesson4.test_ir_read
"""

from pathlib import Path
from dvisd_autonomy.utils import load_yaml
import time
import RPi.GPIO as GPIO


def main(config_path):

    # Load config (kept for consistency)
    config = load_yaml(config_path)

    IR_PIN = 21
    BLACK_STATE = 1   # change to 0 if logic inverted

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("IR sensor test started.")
    print("Move sensor over black tape and white floor.")
    print("Press Ctrl+C to exit.\n")

    try:
        while True:
            state = GPIO.input(IR_PIN)

            if state == BLACK_STATE:
                print("BLACK (state = {})".format(state))
            else:
                print("WHITE (state = {})".format(state))

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nTest stopped by user.")

    finally:
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))