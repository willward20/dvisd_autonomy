from dvisd_autonomy.hardware.control import Control
from dvisd_autonomy.utils import load_yaml
from pathlib import Path
import time
from dvisd_autonomy.hardware.ir import IR

"""Drive forward until a black line is detected by an IR sensor."""

def main(config_path):
    
    # Load configuration for the robot
    config = load_yaml(config_path)

    # Initialize the motors. Motors automatically set to neutral.
    print("Initializing motors...")
    control = Control(**config["control"])

    # Create IR sensor object
    ir_sensor = IR(21) 

    BLACK_STATE = 1         
    N_CONSEC = 3            

    print("Driving forward until black line detected...")
    control.forward(1650)

    count = 0
    while True:
        state = ir_sensor.get_ir_state()
        print(state)

        if state == BLACK_STATE:
            count += 1
        else:
            count = 0

        if count >= N_CONSEC:
            print("Black line detected. Stopping.")
            control.stop()
            break

        time.sleep(0.02)


if __name__ == "__main__":

    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"

    main(str(config_path))
