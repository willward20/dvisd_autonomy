from adafruit_servokit import ServoKit
import time

STEER_CH0 = 0
STEER_CH1 = 1
THROTTLE_CH = 2

kit = ServoKit(channels=16)
kit.frequency = 50
# kit.servo[THROTTLE_CH].set_pulse_width_range(300, 450)

def neutral():

    """ Send neutral pulse (stop) on startup and shutdown. """

    print("Resetting to neutral.")
    kit.servo[THROTTLE_CH].fraction = 0.53
    kit.servo[STEER_CH0].angle = 90
    kit.servo[STEER_CH1].angle = 90
    time.sleep(5)

def turn(angle):

    """ Set the steering angle for both servos (clamped between 0 and 180) """

    clamped_angle = min(180, max(0, angle))
    print(f"Turning {clamped_angle} degrees.")

    kit.servo[STEER_CH0].angle = clamped_angle
    kit.servo[STEER_CH1].angle = clamped_angle

def forward():

    """ Set a constant forward throttle. """

    print("Moving forward.")
    kit.servo[THROTTLE_CH].fraction = 0.60


# Always set vehicle to neutral on startup (stop and wheels centered).
neutral()


# --------------------------
# Write your code here....

# Straight forward.
forward()
time.sleep(2)

# Turn and drive forward.
forward()
turn(70)
time.sleep(2)

# Turn only.
turn(110)
time.sleep(2)
# --------------------------


# Always stop the vehicle on shutdown
neutral()