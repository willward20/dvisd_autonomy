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

    # Turn left
    print("Turning")
    control.turn(120)
    time.sleep(2)

    # Moving straight
    print("Moving straight")
    control.turn(90)
    control.forward()
    time.sleep(2)

    # -----------------------------------


    # Always reset to neutral
    print("Reseting motors to neutral...")
    control.stop()


if __name__ == "__main__":

    config_path = Path.home() / "dvisd_autonomy/config/cardinal2.yaml"

    main(str(config_path))