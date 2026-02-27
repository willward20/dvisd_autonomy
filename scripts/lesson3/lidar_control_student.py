import numpy as np
import time
from pathlib import Path
from rplidar import RPLidar
from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml

# ==============================================================================
#  STUDENT SECTION
# ==============================================================================

# 1. CONFIGURATION
# ----------------
# Width of the cone to check (e.g., 60 means 30 left and 30 right)
FAN_ANGLE = 180
STOP_DISTANCE_MM = 1000

# 2. CALCULATE ARRAY INDICES (THE WRAPPING PROBLEM)
# -------------------------------------------------
# The Lidar array is [0, 1, ..., 359]. 
# Index 0 is directly FRONT.
# To see a cone at the front, we need two slices of this array:
#   - Slice A: From 0 going UP (Robot's Left)
#   - Slice B: From 360 going DOWN (Robot's Right)

half_angle = int(FAN_ANGLE / 2)

# Define the start and end indices for the LEFT side (0 to +angle)
# Python slice syntax: array[ start : end ]
LEFT_IDX_START = 0
LEFT_IDX_END   = 90   # <--- FIX THIS

# Define the start and end indices for the RIGHT side (360 down to -angle)
# HINT: The array ends at 360. You need to back up by 'half_angle'.
RIGHT_IDX_START = 270   # <--- FIX THIS
RIGHT_IDX_END   = 360

# ==============================================================================
#  MAIN LOGIC (Do not change)
# ==============================================================================

def main(config_path):
    config = load_yaml(config_path)
    control = Control(**config["control"])
    lidar = RPLidar("/dev/ttyUSB0")
    
    scan_data = np.zeros(360)

    try:
        print(f"Driving forward. Checking {FAN_ANGLE} degree cone...")
        print(f"Left Slice:  [{LEFT_IDX_START}:{LEFT_IDX_END}]")
        print(f"Right Slice: [{RIGHT_IDX_START}:{RIGHT_IDX_END}]")
        
        control.forward(1650)

        for scan in lidar.iter_scans():
            # Populate scan data
            for (_, angle, distance) in scan:
                scan_angle = min(359, int(angle % 360))
                scan_data[scan_angle] = distance

            # --- Slicing Logic using Student Variables ---
            left_view  = scan_data[LEFT_IDX_START : LEFT_IDX_END]
            right_view = scan_data[RIGHT_IDX_START : RIGHT_IDX_END]

            # Combine views
            front_cone = np.concatenate((left_view, right_view))

            # Filter valid points
            valid_distances = front_cone[front_cone > 0]

            if len(valid_distances) > 0:
                # 10th percentile for safety
                closest_obj = np.percentile(valid_distances, 10)
                
                print(f"Dist: {closest_obj:.1f} mm")

                if closest_obj <= STOP_DISTANCE_MM:
                    print("Wall detected! Stopping.")
                    control.stop()
                    break
            
    except Exception as e:
        print(f"Error: {e}")

    finally:
        print("Shutting down...")
        lidar.stop()
        lidar.disconnect()
        control.shutdown()

if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))
