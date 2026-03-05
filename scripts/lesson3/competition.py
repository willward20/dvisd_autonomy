import time
import RPi.GPIO as GPIO
from pathlib import Path
from dvisd_autonomy.control import Control
from dvisd_autonomy.utils import load_yaml

# ==============================================================================
#  HARDWARE SETUP (Do not change)
# ==============================================================================

TRIG = 20
ECHO = 12
SPEED_OF_SOUND = 34300 # cm/s

# Motor Speeds (PWM values)
# Neutral seems to be around 1600
# >1600 is Forward, <1600 is Backward.
PWM_FAST_FWD = 1665
PWM_SLOW_FWD = 1650
PWM_FAST_REV = 1480
PWM_SLOW_REV = 1495

config_path = Path.home() / "dvisd_autonomy/config/cardinal1.yaml"
config = load_yaml(str(config_path))
control = Control(**config["control"])

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)
time.sleep(0.5) # Let sensors settle

def get_distance_cm():
    """Reads the sonar sensor and returns distance in cm."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start = time.time()
    timeout = start + 0.04
    
    # Wait for Echo High
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        start = time.time()
    
    # Wait for Echo Low
    stop = time.time()
    timeout = stop + 0.04
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        stop = time.time()

    pulse = stop - start
    distance = (pulse * SPEED_OF_SOUND) / 2

    # Basic filter: ignore crazy values (0 or > 500)
    if distance < 2 or distance > 500:
        return 999

    # print(distance)
    return distance

# ==============================================================================
#  STUDENT API FUNCTIONS
# ==============================================================================

def forward_fast(stop_at_dist):
    """Drives FAST until distance is LESS than stop_at_dist."""
    print(f"--> Forward FAST until {stop_at_dist}cm")
    control.forward(PWM_FAST_FWD)
    start = time.time()
    while time.time() - start < 1:
        d = get_distance_cm()
        if d < stop_at_dist:
            print("FF", d)
            control.stop()
            break
        time.sleep(0.02)

def forward_slow(stop_at_dist):
    """Drives SLOW until distance is LESS than stop_at_dist."""
    print(f"--> Forward SLOW until {stop_at_dist}cm")
    control.forward(PWM_SLOW_FWD)
    start = time.time()
    while time.time() - start < 1:
        d = get_distance_cm()
        if d < stop_at_dist:
            print("FS", d)
            control.stop()
            break
        time.sleep(0.02)

def backward_fast(stop_at_dist):
    """Drives BACKWARD until distance is GREATER than stop_at_dist."""
    print(f"<-- Backward FAST until {stop_at_dist}cm")
    control.forward(PWM_FAST_REV)
    start = time.time()
    while time.time() - start < 1:
        d = get_distance_cm()
        if d > stop_at_dist:
            print("BF", d)
            control.stop()
            break
        time.sleep(0.02)

def backward_slow(stop_at_dist):
    """Drives BACKWARD until distance is GREATER than stop_at_dist."""
    print(f"<-- Backward SLOW until {stop_at_dist}cm")
    control.forward(PWM_SLOW_REV)
    start = time.time()
    while time.time() - start < 1:
        d = get_distance_cm()
        if d > stop_at_dist:
            print("BS", d)
            control.stop()
            break
        time.sleep(0.02)

def stop_car():
    """Immediately stops the motors."""
    print("--- STOP ---")
    control.stop()

# ==============================================================================
#  STUDENT STRATEGY SECTION
#  Edit the code below to win the competition!
# ==============================================================================

try:
    print("Starting Run...")
    
    # EXAMPLE STRATEGY:
    # 1. Rush close to the wall (stop at 150cm)
    forward_fast(150)   # 1 sec each
    forward_fast(150)

    # 2. Creep closer (stop at 120cm to account for momentum!)
    forward_slow(120)

    time.sleep(1)

    # 3. If we overshot (too close), back up a little?
    backward_slow(100) 

    # Final stop
    stop_car()

    # Verify Final Distance
    time.sleep(1)
    final_dist = get_distance_cm()
    print(f"FINAL SCORE: {final_dist:.1f} cm")

except KeyboardInterrupt:
    control.stop()
    GPIO.cleanup()

finally:
    control.shutdown()
    GPIO.cleanup()
