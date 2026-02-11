from adafruit_servokit import ServoKit
import time


STEER_CH0 = 0
STEER_CH1 = 1
THROTTLE_CH = 2


class Control:

    """ A class for sending control signals to an RC car's steering and throttle motors. """

    def __init__(
            self, 
            freq_hz: int, 
            steering_pw_min: int, 
            steering_pw_max: int, 
            throttle_pw_min: int, 
            throttle_pw_max: int,
            neutral_angle: int,
            neutral_fraction: float 
        ):

        """ Initialize the control interface and set motors to neutral. """

        # Configure the ServoKit interface.
        self.kit = ServoKit(channels=16)
        self.kit.freq_hz = freq_hz
        self.kit.servo[THROTTLE_CH].set_pulse_width_range(throttle_pw_min, throttle_pw_max)
        self.kit.servo[STEER_CH0].set_pulse_width_range(steering_pw_min, steering_pw_max)
        self.kit.servo[STEER_CH1].set_pulse_width_range(steering_pw_min, steering_pw_max)

        # Set the neutral values
        self.neutral_angle = neutral_angle
        self.neutral_fraction = neutral_fraction

        # Initialize the motors and reset steering angle.
        self.stop()
        time.sleep(5)


    def turn(self, angle: int):

        """ Set the steering angle. """

        clamped_angle = min(140, max(50, angle))

        self.kit.servo[STEER_CH0].angle = clamped_angle
        self.kit.servo[STEER_CH1].angle = clamped_angle


    def forward(self, fraction: float = 0.6):

        """ Set the forward throttle fraction. """

        self.kit.servo[THROTTLE_CH].fraction = fraction


    def straight(self):

        """ Reset steering angle to center. """

        self.turn(self.neutral_angle)


    def stop(self):

        """ Reset throttle and steering to neutral values. """

        self.forward(self.neutral_fraction)
        self.turn(self.neutral_angle)