import math
import numpy as np

class PurePursuitController:
    def __init__(self, vehicle_control, wheelbase: float, lookahead_dist: float):
        """
        :param vehicle_control: Instance of your Control class
        :param wheelbase: Distance between front and rear axles (meters)
        :param lookahead_dist: Distance to look ahead on the path (meters)
        """
        self.car = vehicle_control
        self.L = wheelbase
        self.ld = lookahead_dist

    def find_target_point(self, curr_x, curr_y, path):
        """
        Finds the point on the path that is closest to the lookahead distance.
        Path should be a list/array of (x, y) coordinates.
        """
        distances = np.sqrt(np.sum((path - np.array([curr_x, curr_y]))**2, axis=1))
        
        # Find points that are at least 'ld' away
        valid_indices = np.where(distances >= self.ld)[0]
        
        if len(valid_indices) == 0:
            return path[-1]  # Default to the last point
        
        return path[valid_indices[0]]

    def update(self, curr_x, curr_y, curr_yaw, path):
        """
        Calculates the required steering angle and applies it.
        :param curr_yaw: Heading in radians
        """
        # 1. Get target point
        target_x, target_y = self.find_target_point(curr_x, curr_y, path)

        # 2. Transform target to vehicle coordinates
        dx = target_x - curr_x
        dy = target_y - curr_y
        
        # Local relative coordinates
        local_x = dx * math.cos(-curr_yaw) - dy * math.sin(-curr_yaw)
        local_y = dx * math.sin(-curr_yaw) + dy * math.cos(-curr_yaw)

        # 3. Calculate Steering Angle (delta)
        # Formula: delta = atan2(2 * L * sin(alpha) / ld)
        # In local coords, sin(alpha) = local_y / dist_to_target
        dist_sq = local_x**2 + local_y**2
        steering_angle_rad = math.atan2(2 * self.L * local_y, dist_sq)
        
        # 4. Convert to Degrees for your API
        steering_angle_deg = self.car.neutral_angle + math.degrees(steering_angle_rad)
        
        # 5. Apply to hardware
        self.car.turn(int(steering_angle_deg))
        self.car.forward() # Maintain constant speed