import math
import time

class DeadReckoner:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, wheelbase=0.25):
        """
        :param x, y: Initial position in meters
        :param yaw: Initial heading in radians (0 = facing East/X-axis)
        :param wheelbase: Distance between front and rear axles (L)
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.L = wheelbase
        self.last_time = time.time()

    def update(self, velocity, steering_angle_deg, neutral_angle):
        """
        Updates the internal state based on current movement.
        :param velocity: Estimated speed in m/s
        :param steering_angle_deg: The current angle of the servo
        :param neutral_angle: The 'straight' angle of your servo
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 1. Convert servo angle to steering angle (delta) in radians
        # Positive = Left turn, Negative = Right turn
        delta = math.radians(steering_angle_deg - neutral_angle)

        # 2. Kinematic Bicycle Model Equations
        # Change in position
        dx = velocity * math.cos(self.yaw) * dt
        dy = velocity * math.sin(self.yaw) * dt
        
        # Change in heading (yaw)
        # Formula: d_yaw = (v / L) * tan(delta)
        dyaw = (velocity / self.L) * math.tan(delta) * dt

        # 3. Update state
        self.x += dx
        self.y += dy
        self.yaw += dyaw
        
        # Keep yaw within [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        return self.x, self.y, self.yaw