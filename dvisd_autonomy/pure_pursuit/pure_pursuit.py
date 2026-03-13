import math
import numpy as np

class PurePursuitController:
    def __init__(self, vehicle_control, wheelbase: float, lookahead_dist: float):
        self.car = vehicle_control
        self.L = wheelbase
        self.ld = lookahead_dist
        self.last_idx = 0  # Track progress to ensure we only look forward

    def find_target_point(self, curr_x, curr_y, path):
        """
        Finds the lookahead point by searching only forward from the last found index.
        """
        num_points = len(path)
        
        # 1. Start searching from the last known index
        # This prevents the car from 'looking back' at previous segments
        relevant_path = path[self.last_idx:]
        
        # Calculate Euclidean distances from current position to all forward points
        # Using squared distance is slightly faster for computation
        dists = np.sqrt(np.sum((relevant_path - np.array([curr_x, curr_y]))**2, axis=1))
        
        # 2. Find the first point that is at least 'ld' distance away
        found_new_idx = False
        for i in range(len(dists)):
            if dists[i] >= self.ld:
                self.last_idx = self.last_idx + i
                found_new_idx = True
                break
        
        # 3. If no point is far enough away, we are likely near the end
        if not found_new_idx:
            self.last_idx = num_points - 1
        print("Subgoal:", path[self.last_idx])
        return path[self.last_idx]

    def update(self, curr_x, curr_y, curr_yaw, path):
        """
        Calculates the required steering angle and applies it.
        """
        # 1. Get target point (now strictly forward-moving)
        target_x, target_y = self.find_target_point(curr_x, curr_y, path)
        
        # 2. Transform target to vehicle coordinates
        dx = target_x - curr_x
        dy = target_y - curr_y
        
        # Rotate coordinates into the car's local frame
        # x-forward, y-left
        local_x = dx * math.cos(-curr_yaw) - dy * math.sin(-curr_yaw)
        local_y = dx * math.sin(-curr_yaw) + dy * math.cos(-curr_yaw)

        # 3. Calculate Steering Angle (delta)
        # Using the simplified geometric relation for the bicycle model
        dist_sq = local_x**2 + local_y**2
        steering_angle_rad = math.atan2(2 * self.L * local_y, dist_sq)
        
        # 4. Convert to Degrees and clamp/offset based on hardware
        steering_angle_deg = self.car.neutral_angle + math.degrees(steering_angle_rad)
        
        # 5. Apply to hardware
        print("Steering angle", int(steering_angle_deg))
        self.car.turn(int(steering_angle_deg))
        # Note: You might want to pass the specific pulse_us to forward() 
        # instead of the default to maintain consistent speed
        self.car.forward()