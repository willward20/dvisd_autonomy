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


# def best_direction(ranges, angles):
#     """
#     Compute a desired direction vector given lidar ranges.
#     ranges: array of distances (meters) for wedge angles
#     Returns: angle in radians
#     """
#     v = 2.0 * np.array([1.0, 0.0])  # initial bias forward

#     for r, theta in zip(ranges, angles):
#         if r > 3.0 or r == 0.0:
#             print("SKIP")
#             continue
#         print("r, theta: ", r, theta*180/np.pi)
#         r = max(r, 0.2)
#         print("r: ", r)
#         w = 1.0 / (r * r)
#         vx = -w * np.cos(theta)
#         vy = -w * np.sin(theta)
#         print("vx, vy: ", vx, vy)
#         v += np.array([vx, vy])
#     print("v: ", v)
#     angle = np.arctan2(v[1], v[0])
#     return angle * 180 / np.pi

import matplotlib.pyplot as plt

def best_direction(ranges, angles, plot=True):
    """
    Compute a desired direction vector given lidar ranges and angles.
    Optionally plot the points and vectors.
    Returns: angle in degrees
    """
    v = np.array([2.0, 0.0])  # initial bias forward
    vectors = []

    for r, theta in zip(ranges, angles):
        if r > 3.0 or r == 0.0:
            continue
        r = max(r, 0.2)
        w = 1.0 / (r * r)
        vx = -w * np.cos(theta)
        vy = -w * np.sin(theta)
        vectors.append((vx, vy))
        v += np.array([vx, vy])

    angle = np.arctan2(v[1], v[0])
    
    if plot:
        _plot_lidar_vectors(ranges, angles, vectors, v)

    return angle


def _plot_lidar_vectors(ranges, angles, vectors, result_vector):
    """
    Simple matplotlib plot showing:
    - lidar points
    - individual repulsion vectors
    - resulting vector
    """
    plt.figure(figsize=(6, 6))
    ax = plt.gca()
    ax.set_aspect('equal')

    # Plot lidar points
    x_points = ranges * np.cos(angles)
    y_points = ranges * np.sin(angles)
    ax.scatter(x_points, y_points, c='blue', s=10, label='Lidar points')

    # Plot individual vectors
    origin_x = np.zeros(len(vectors))
    origin_y = np.zeros(len(vectors))
    vec_x = [vx for vx, vy in vectors]
    vec_y = [vy for vx, vy in vectors]
    ax.quiver(origin_x, origin_y, vec_x, vec_y, color='red', angles='xy', scale_units='xy', scale=1, alpha=0.5)

    # Plot resulting vector
    ax.quiver(0, 0, result_vector[0], result_vector[1], color='green', scale_units='xy', scale=1, width=0.01, label='Result vector')

    # Plot robot forward direction
    ax.arrow(0, 0, 2.0, 0, color='black', width=0.02, head_width=0.1, label='Forward')

    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Lidar points and resulting vectors')
    ax.grid(True)
    ax.legend()
    plt.savefig("scan.png")


def main(config_path):
    # Load configuration
    config = load_yaml(config_path)

    # Initialize hardware
    control = Control(**config["control"])
    lidar = Lidar()

    control.forward(1650)

    for scan_data, angles in lidar.get_scans():
        # Front wedge: 45 degree angle from forward
        wedge = np.concatenate((scan_data[:45], scan_data[315:]))
        wedge_angles = np.concatenate((angles[:45], angles[315:] - 2*np.pi))
        # print("wedge angles: ", wedge_angles)
        desired_angle = best_direction(wedge, wedge_angles)

        # Controller outputs desired steering angle
        delta_desired = np.clip(desired_angle, -MAX_STEER_RAD, MAX_STEER_RAD)

        # Convert to real robot command
        servo_cmd = steering_to_servo(delta_desired)
        print("servo_cmd: ", servo_cmd)

        control.turn(servo_cmd)
        time.sleep(0.1)

        return
            

if __name__ == "__main__":
    config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
    main(str(config_path))