from dvisd_autonomy.pure_pursuit.navigator import AutonomousNavigator
from dvisd_autonomy.control.control import Control
import time
import numpy as np

def generate_arc_from_origin(radius=3.0, sweep_deg=90, num_points=30):
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
### Tune the parameters yourself ###
esc_neutral_us = 1580
esc_forward_us = 1660
wheelbase = 0.2
lookahead = 0.1
resolution = 0.05
METERS_PER_SEC_PER_US = 0.05
####################################

# 1. Define your path: A simple 2-meter square
square_waypoints = [
    [0.0, 0.0],
    [3.0, 0.0],
    [4.0, 4.0],
]
square_waypoints = generate_arc_from_origin(radius=3.0, sweep_deg=180, num_points=3)

# 2. Initialize your specific hardware config
rc_hardware = Control(
    freq_hz=100,
    esc_neutral_us=esc_neutral_us,
    esc_forward_us=esc_forward_us, # Slow crawl for testing
    neutral_angle=100,
    steering_min=50,
    steering_max=140
)

# 3. Start the navigator
nav = AutonomousNavigator(
    rc_hardware,
    square_waypoints,
    wheelbase=wheelbase,
    lookahead=lookahead,
    resolution=resolution,
    esc_neutral_us=esc_neutral_us,
    METERS_PER_SEC_PER_US=METERS_PER_SEC_PER_US
)
nav.run(target_pulse_us=esc_forward_us)