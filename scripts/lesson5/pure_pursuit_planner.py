"""
source ~/myvenv/bin/activate
python -m scripts.lesson5.pure_pursuit_planner
"""

from dvisd_autonomy.pure_pursuit.navigator import AutonomousNavigator
from dvisd_autonomy.control.control import Control
import time
import numpy as np

def generate_arc_from_origin(radius=3.0, sweep_deg=45, num_points=30):
    """
    Generates an arc starting at (0,0).
    Default: Sweeps 90 degrees upward (Left Turn).
    """
    # Center is at (0, radius) so that (0,0) is on the edge
    center_x = 0
    center_y = radius
    
    # Start at -90 degrees (bottom of circle) to hit (0,0)
    start_angle = np.radians(-90)
    end_angle = np.radians(-90 + sweep_deg)
    angles = np.linspace(start_angle, end_angle, num_points)

    arc = []
    for theta in angles:
        x = center_x + radius * np.cos(theta)
        y = center_y + radius * np.sin(theta)
        arc.append([x, y])
        
    return np.array(arc)

def generate_lane_change(run_up_length = 5, num_changes = 1, length_scale = 1.0):

    """
        Generate a path comprising num_changes lane switches
        with a certain length_scale
    """ 

    arc_waypoints = [[0,0]]
    arc_waypoints.append([run_up_length, 0])
    current_x = run_up_length
    current_y = 0

    # repeat a manuever of the form /^\_ num_changes times
    for i in range(num_changes):

        current_x = current_x + length_scale
        current_y = current_y + length_scale

        arc_waypoints.append([current_x, current_y])

        current_x = current_x + 2*length_scale

        arc_waypoints.append([current_x, current_y])

        current_x = current_x + length_scale
        current_y = current_y - length_scale

        arc_waypoints.append([current_x, current_y])


        current_x += 4*length_scale
        arc_waypoints.append([current_x, current_y])

    
    return np.array(arc_waypoints)

def add_run_up(path, run_up_length = 1.0):

    """
        Given a path, add a run_up of specified length
        to the path
    """

    L = np.shape(path)[0]

    new_path = np.zeros((L+1, 2))

    new_path[0] = [0, 0]

    for i in range(L):
        new_path[i+1] = path[i] + [run_up_length, 0]

    return new_path


### Tune the parameters yourself ###
esc_neutral_us = 1580
esc_forward_us = 1645
wheelbase = 0.3
lookahead = 0.1
resolution = 0.001
meters_per_sec_per_us = 0.02
####################################

# 1. Define a simple arc, with a run-up to get to a good speed
arc_waypoints = generate_arc_from_origin(radius=2.0, sweep_deg=180, num_points=3)
arc_waypoints = add_run_up(arc_waypoints, run_up_length=2.0)

arc_waypoints = generate_lane_change(run_up_length=2, num_changes = 1, length_scale = 1)

print("WAYPOINTS = ", arc_waypoints)

# 2. Initialize your specific hardware config
rc_hardware = Control(
    freq_hz=100,
    esc_neutral_us=esc_neutral_us,
    esc_forward_us=esc_forward_us, 
    neutral_angle=100,
    steering_min=50,
    steering_max=140
)

# 3. Start the navigator
nav = AutonomousNavigator(
    rc_hardware,
    arc_waypoints,
    wheelbase=wheelbase,
    lookahead=lookahead,
    resolution=resolution,
    esc_neutral_us=esc_neutral_us,
    meters_per_sec_per_us=meters_per_sec_per_us
)
nav.run(target_pulse_us=esc_forward_us)