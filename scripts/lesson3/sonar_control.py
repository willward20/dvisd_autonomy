from dvisd_autonomy.control.control import Control
from dvisd_autonomy.control.utils import load_yaml
from dvisd_autonomy.sensors.sonar import Sonar
from pathlib import Path
import time


STOP_DISTANCE_CM = 50
SPEED_OF_SOUND = 34300  # cm/s


def get_distance_cm(sonar):
    sonar.send_pulse()

    start = time.time()
    timeout = start + 0.02

    while sonar.get_sonar_signal() == 0 and time.time() < timeout:
        start = time.time()

    stop = time.time()
    timeout = stop + 0.02

    while sonar.get_sonar_signal() == 1 and time.time() < timeout:
        stop = time.time()

    pulse = stop - start
    return (pulse * SPEED_OF_SOUND) / 2


def main(config_path):

    config = load_yaml(config_path)

    print("Initializing motors...")
    control = Control(**config["control"])

    sonar = Sonar(20, 12)

    print("Driving forward until wall detected...")
    control.forward(1650)

    while True:
        distance = get_distance_cm(sonar)
        print(f"{distance:.1f} cm")

        if distance <= STOP_DISTANCE_CM:
            print("Wall detected. Stopping.")
            control.stop()
            break

        time.sleep(0.05)


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))
