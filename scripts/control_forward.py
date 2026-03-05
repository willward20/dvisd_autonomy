from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml
from pathlib import Path
import time


def main(config_path):
    
    # Load configuration for the robot
    config = load_yaml(config_path)

    # Initialize the motors. Motors automatically set to neutral.
    print("Initializing motors...")
    control = Control(**config["control"])


    # ------ WRITE YOUR CODE HERE -------

    # small forward
    print("small forward")
    control.forward(1650)
    time.sleep(2.0)
    control.stop()

    # control.turn(100)
    # time.sleep(0.5)

    # control.turn(80)
    # time.sleep(0.25)
    # control.turn(120)
    # time.sleep(0.25)
    # control.turn(80)
    # time.sleep(0.25)
    # control.turn(120)
    # time.sleep(0.25)

    # control.turn(100)
    # time.sleep(0.5)

    # -----------------------------------


    # Always reset to neutral
    print("Reseting motors to neutral...")
    control.shutdown()


if __name__ == "__main__":

    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"

    main(str(config_path))
