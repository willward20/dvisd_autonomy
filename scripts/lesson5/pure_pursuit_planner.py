from dvisd_autonomy.pure_pursuit.navigator import AutonomousNavigator
from dvisd_autonomy.control.control import Control

### Tune the parameters yourself ###
esc_neutral_us = 1580
esc_forward_us = 1650
wheelbase = 0.2
lookahead = 0.4
resolution = 0.05
METERS_PER_SEC_PER_US = 0.01
####################################

# 1. Define your path: A simple 2-meter square
square_waypoints = [
    [0.0, 0.0],
    [2.0, 0.0],
    [2.0, 2.0],
    [0.0, 2.0],
    [0.0, 0.0]
]

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