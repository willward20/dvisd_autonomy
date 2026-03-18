import time
import numpy as np
from pathlib import Path
from dvisd_autonomy.control.control import Control
from dvisd_autonomy.utils import load_yaml
from dvisd_autonomy.sensors.lidar import Lidar


SERVO_CENTER = 100
SERVO_MIN = 50
SERVO_MAX = 140
MAX_STEER_RAD = 0.5  # same as your current clip (~28.6 deg)


def steering_to_servo(delta):
    """
    Convert steering angle (radians) → servo command
    """
    normalized = np.clip(delta / MAX_STEER_RAD, -1.0, 1.0)
    cmd = SERVO_CENTER + normalized * (SERVO_MAX - SERVO_CENTER)
    return np.clip(cmd, SERVO_MIN, SERVO_MAX)


def best_direction(ranges, angles):
    """
    Compute a desired direction vector given lidar ranges and angles.
    Optionally plot the points and vectors.
    Returns: angle in radians
    """
    v = np.array([1.0, 0.0])  # initial bias forward
    vectors = []

    for r, theta in zip(ranges, angles):
        if r > 3.0 or r == 0.0:
            continue
        r = max(r, 0.2)

        x = np.cos(theta)
        y = np.sin(theta)

        w = 0.5 / (r*r)
     
        if y >= 0:
            vx = w * y
            vy = - w * x
        else:
            vx = - w * y
            vy = w * x

        # w = 0.01 / (r * r)
        # vx = -w * np.cos(theta)
        # vy = -w * np.sin(theta)
        vectors.append((vx, vy))
        v += np.array([vx, vy])

    return np.arctan2(v[1], v[0])






def find_best_gap(ranges, min_dist=1.0):
    """
    Find largest contiguous region where range > min_dist
    """
    valid = ranges > min_dist

    gaps = []
    start = None

    for i, v in enumerate(valid):
        if v and start is None:
            start = i
        elif not v and start is not None:
            gaps.append((start, i))
            start = None

    if start is not None:
        gaps.append((start, len(ranges)))

    if not gaps:
        return None

    return max(gaps, key=lambda g: g[1] - g[0])


def select_goal_point(ranges, angles, gap):
    """
    Pick midpoint of gap as goal
    """
    start, end = gap
    mid = (start + end) // 2

    r = ranges[mid]
    theta = angles[mid]

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    return np.array([x, y])


def pure_pursuit_angle(goal_point):
    """
    Steering angle toward goal point (robot frame)
    """
    return np.arctan2(goal_point[1], goal_point[0])



def main(config_path):
    # Load configuration
    config = load_yaml(config_path)

    # Initialize hardware
    control = Control(**config["control"])
    lidar = Lidar()

    # Front wedge angle from center
    PSI = 45 

    # control.forward(1650)

    for scan_data, angles in lidar.get_scans():

        wedge = np.concatenate((scan_data[:PSI], scan_data[(360-PSI):]))
        wedge_angles = np.concatenate((angles[:PSI], angles[(360-PSI):] - 2*np.pi))
        # desired_angle = best_direction(wedge, wedge_angles)
        gap = find_best_gap(wedge)

        if gap is not None:
            goal = select_goal_point(wedge, wedge_angles, gap)
            desired_angle = pure_pursuit_angle(goal)
        else:
            desired_angle = 0.0  # fallback

        print("Desired angle (radians): ", desired_angle)
        print("Desired angle (degrees): ", desired_angle * 180 / np.pi)

        # Controller outputs desired steering angle
        delta_desired = np.clip(desired_angle, -MAX_STEER_RAD, MAX_STEER_RAD)

        # Convert to real robot command
        servo_cmd = steering_to_servo(delta_desired)
        print("Servo command: ", servo_cmd)
        control.turn(servo_cmd)


if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))