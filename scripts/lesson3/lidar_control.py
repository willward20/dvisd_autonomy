import numpy as np
from pathlib import Path
from dvisd_autonomy.hardware.control import Control
from dvisd_autonomy.utils import load_yaml
from dvisd_autonomy.hardware.lidar import Lidar


def main(config_path):
    # Load configuration
    config = load_yaml(config_path)

    # Initialize hardware
    control = Control(**config["control"])
    lidar = Lidar()

    # Lidar setup (now returns meters)
    STOP_DISTANCE_M = 1.0  # 1 meter

    print("Driving forward until wall detected...")
    control.forward(1650)

    for scan_data in lidar.get_scans():
        # Front wedge: 0–20 and 340–360
        front_angles = np.concatenate((scan_data[:20], scan_data[340:]))

        # Filter invalid readings (0.0 means no return)
        valid_distances = front_angles[front_angles > 0]

        if valid_distances.size == 0:
            continue

        # Robust distance estimate
        dist_10th = np.percentile(valid_distances, 10)

        print(f"Front 10th Percentile: {dist_10th:.2f} m")

        if dist_10th <= STOP_DISTANCE_M:
            print("Wall detected. Stopping.")
            control.stop()
            break


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))