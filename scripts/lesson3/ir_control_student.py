from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml
from pathlib import Path
import time
import RPi.GPIO as GPIO

def main(config_path):
    
    # Load configuration for the robot
    config = load_yaml(config_path)

    # Initialize the motors. Motors automatically set to neutral.
    print("Initializing motors...")
    control = Control(**config["control"])


    # Execution code
    IR_PIN = 21

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # ----- Insert the state value -----
    BLACK_STATE =          
    N_CONSEC = 3            

    print("Driving forward until black line detected...")
    control.forward(1650)

    count = 0
    try:
        while True:
            state = GPIO.input(IR_PIN)
            print(state)

            # ----- Insert the if logic -----
            if 
            # -------------------------------
                count += 1
            else:
                count = 0

            if count >= N_CONSEC:
                print("Black line detected. Stopping.")
                control.stop()
                break

            time.sleep(0.02)

    finally:
        GPIO.cleanup()
    # -----------------------------------
    # Always reset to neutral
    print("Reseting motors to neutral...")
    control.shutdown()


if __name__ == "__main__":

    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"

    main(str(config_path))
