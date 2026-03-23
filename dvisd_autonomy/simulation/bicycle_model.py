import numpy as np


class BicycleModel:
    def __init__(
        self,
        wheelbase=0.2,
        dt=0.05,
        speed=0.5,
        servo_center=100,
        servo_min=50,
        servo_max=140,
    ):
        self.L = wheelbase
        self.dt = dt
        self.v = speed

        # Servo parameters
        self.servo_center = servo_center
        self.servo_min = servo_min
        self.servo_max = servo_max


    def _steering_to_servo(self, delta):
        """Convert steering angle (rad) → servo command (deg)"""
        angle = np.degrees(delta) + self.servo_center
        return min(self.servo_max, max(self.servo_min, angle))


    def _servo_to_delta(self, angle):
        """Convert servo command → steering angle (rad)"""
        angle -= self.servo_center
        return np.radians(angle)
        

    def _kbm_step(self, state, delta):
        """One timestep of bicycle dynamics."""
        x, y, psi = state

        x += self.v * np.cos(psi) * self.dt
        y += self.v * np.sin(psi) * self.dt
        psi += (self.v / self.L) * np.tan(delta) * self.dt

        return np.array([x, y, psi])


    def step(self, state, angle):
        """
        Simulates real system:
        desired steering → servo angle → dynamics

        state: [x, y, psi]
        angle: desired steering angle (rad)
        """
        servo_angle = self._steering_to_servo(angle)
        delta = self._servo_to_delta(servo_angle)
        return self._kbm_step(state, delta)