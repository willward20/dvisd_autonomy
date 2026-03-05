import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


STEER_CH0 = 0
STEER_CH1 = 1
THROTTLE_CH = 2


class Control:

    """ Control interface for RC car using PCA9685. """

    def __init__(
        self,
        freq_hz: int,
        esc_neutral_us: int,
        esc_forward_us: int,
        neutral_angle: int,
        steering_min: int,
        steering_max: int,
    ):

        """ Initialize the control interface and set motors to neutral. """

        # Store config
        self.esc_neutral_us = esc_neutral_us
        self.esc_forward_us = esc_forward_us
        self.neutral_angle = neutral_angle
        self.steering_min = steering_min
        self.steering_max = steering_max

        # Derived timing
        self.period_us = int(1_000_000 / freq_hz)

        # Initialize I2C + PCA
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = freq_hz

        # Steering servo object
        self.steering = servo.Servo(self.pca.channels[STEER_CH1])

        # Safety neutral on startup
        self.stop()
        self.straight()
        time.sleep(5)


    def _us_to_counts(self, pulse_us: int) -> int:

        """ Convert microsecond pulse width to PCA9685 counts. """

        pulse_us = max(500, min(2500, pulse_us))
        return int(round(pulse_us * 4096 / self.period_us))


    def _set_pulse_us(self, pulse_us: int):

        """ Set a microsecond pulse on the throttle channel. """

        counts = self._us_to_counts(pulse_us)
        self.pca.channels[THROTTLE_CH].duty_cycle = int(counts * 65535 / 4095)


    def turn(self, angle: int):

        """ Set the steering angle (clamped). """

        clamped = min(self.steering_max, max(self.steering_min, angle))
        self.steering.angle = clamped


    def forward(self, pulse_us: int | None = None):

        """ Set the forward throttle fraction. """

        if pulse_us is None:
            pulse_us = self.esc_forward_us

        self._set_pulse_us(pulse_us)


    def straight(self):

        """ Reset steering angle to center. """

        self.turn(self.neutral_angle)


    def stop(self):

        """ Reset throttle to neutral. """

        self._set_pulse_us(self.esc_neutral_us)


    def shutdown(self):

        """ Safely stop and deinitialize hardware. """

        self.stop()
        self.straight()
        time.sleep(1)
        self.pca.deinit()