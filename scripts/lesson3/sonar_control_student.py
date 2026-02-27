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
	
    # Sonar setup
    TRIG = 20
    ECHO = 12

    # ----- Insert values -----
    STOP_DISTANCE_CM = 
    SONAR_CONSTANT =   # cm/s
    # -------------------------

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, False)
    time.sleep(0.2)

    def get_distance_cm():
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        start = time.time()
        timeout = start + 0.02
        while GPIO.input(ECHO) == 0 and time.time() < timeout:
            start = time.time()

        stop = time.time()
        timeout = stop + 0.02
        while GPIO.input(ECHO) == 1 and time.time() < timeout:
            stop = time.time()

        pulse = stop - start

        # ----- Fill up the equation to find the distance -----
        distance =   # cm
        # -----------------------------------------------------
        
        return distance


    # ---- Drive forward until close to wall ----
    print("Driving forward until wall detected...")
    control.forward(1650)

    while True:
        d = get_distance_cm()
        print(f"{d:.1f} cm")

        if d <= STOP_DISTANCE_CM:
            print("Wall detected. Stopping.")
            control.stop()
            break

        time.sleep(0.05)

    GPIO.cleanup()
    # -----------------------------------


    # Always reset to neutral
    print("Reseting motors to neutral...")
    control.shutdown()


if __name__ == "__main__":

    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"

    main(str(config_path))
