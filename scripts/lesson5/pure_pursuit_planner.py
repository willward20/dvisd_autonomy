from dvisd_autonomy.pure_pursuit_planning.navigator import AutonomousNavigator

# --- EXECUTION BLOCK ---

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
    esc_neutral_us=1580,
    esc_forward_us=1650, # Slow crawl for testing
    neutral_angle=100,
    steering_min=50,
    steering_max=140
)

# 3. Start the navigator
nav = AutonomousNavigator(
    hw,
    square_waypoints,
    wheelbase=0.2,
    lookahead=0.4,
    resolution=0.05
    )
nav.run(target_pulse_us=1580)