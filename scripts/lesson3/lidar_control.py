import numpy as np
import time
from pathlib import Path
from rplidar import RPLidar
from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml

def main(config_path):
    
    # Load configuration for the robot
    config = load_yaml(config_path)

    # Initialize the motors
    print("Initializing motors...")
    control = Control(**config["control"])

    # Initialize Lidar
    print("Initializing Lidar...")
    lidar = RPLidar("/dev/ttyUSB0")
    
    # Lidar Setup
    STOP_DISTANCE_MM = 1000  # 50cm = 500mm
    scan_data = np.zeros(360)

    try:
        # Start driving forward
        print("Driving forward until wall detected...")
        control.forward(1650)

        # Iterate through Lidar scans
        for scan in lidar.iter_scans():
            # Update the scan_data buffer with new points
            for (_, angle, distance) in scan:
                scan_angle = min(359, int(angle % 360))
                scan_data[scan_angle] = distance

            # Extract the "Blue" points (Front facing wedge: 0-20 and 340-360)
            # We combine the left slice (0-20) and the right slice (340-360)
            front_angles = np.concatenate((scan_data[:20], scan_data[340:]))

            # Filter out 0.0 values (invalid or out of range measurements)
            valid_distances = front_angles[front_angles > 0]

            if len(valid_distances) > 0:
                # Compute the 10th percentile distance
                # This finds the value below which 10% of the observations fall
                dist_10th = np.percentile(valid_distances, 10)
                
                print(f"Front 10th Percentile: {dist_10th:.1f} mm")

                # Check if we are close to the wall (100cm / 1000mm)
                if dist_10th <= STOP_DISTANCE_MM:
                    print("Wall detected. Stopping.")
                    control.stop()
                    break
            
    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Always clean up Lidar and reset motors to neutral
        print("Shutting down...")
        lidar.stop()
        lidar.disconnect()
        control.shutdown()

if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))
