import math
import time
import numpy as np
from dvisd_autonomy.pure_pursuit.dead_reckoner import DeadReckoner
from dvisd_autonomy.pure_pursuit.pure_pursuit import PurePursuitController

class AutonomousNavigator:
    def __init__(self, control_api, waypoints, wheelbase=0.25, lookahead=0.5, resolution=0.1, esc_neutral_us=1580, METERS_PER_SEC_PER_US=0.01):
        """
        :param control_api: Instance of your Control class
        :param waypoints: List of [x, y] coordinates
        :param resolution: Max distance between points in meters
        :param esc_neutral_us: Neutral throttle pulse width
        """
        self.car = control_api
        self.wheelbase = wheelbase
        self.lookahead = lookahead
        
        # 1. Automatically upsample the path on initialization
        self.path = self._upsample_path(np.array(waypoints), resolution)
        
        # 2. Initialize Sub-modules
        self.dr = DeadReckoner(x=0.0, y=0.0, yaw=0.0, wheelbase=wheelbase)
        self.pp = PurePursuitController(self.car, wheelbase, lookahead)
        
        # 3. Calibration (Assumed linear mapping)
        self.MIN_THROTTLE_PULSE = esc_neutral_us
        self.METERS_PER_SEC_PER_US = METERS_PER_SEC_PER_US # this is also an estimation

    def _upsample_path(self, path, max_dist):
        """Internal helper to fill gaps between sparse waypoints."""
        upsampled = [path[0]]
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i+1]
            dist = np.linalg.norm(p2 - p1)
            if dist > max_dist:
                num_segments = int(np.ceil(dist / max_dist))
                for j in range(1, num_segments + 1):
                    upsampled.append(p1 + (p2 - p1) * (j / num_segments))
            else:
                upsampled.append(p2)
        return np.array(upsampled)

    def get_velocity_estimate(self, pulse_us):
        if pulse_us <= self.MIN_THROTTLE_PULSE: return 0.0
        return (pulse_us - self.MIN_THROTTLE_PULSE) * self.METERS_PER_SEC_PER_US

    def run(self, target_pulse_us, arrival_threshold=0.15):
        """
        Main loop. 
        :param target_pulse_us: Throttle pulse width to maintain during navigation.
        :param arrival_threshold: Stop if within X meters of the final point.
        """
        print(f"Navigation started. Path contains {len(self.path)} points.")
        
        try:
            while True:
                # 1. Update Position (Dead Reckoning)
                v = self.get_velocity_estimate(target_pulse_us)
                curr_angle = self.car.steering.angle 
                x, y, yaw = self.dr.update(v, curr_angle, self.car.neutral_angle)

                # 2. Check for Goal Completion
                dist_to_goal = np.linalg.norm(np.array([x, y]) - self.path[-1])
                if dist_to_goal < arrival_threshold:
                    print("🏁 Goal reached! Stopping.")
                    break

                # 3. Update Steering (Pure Pursuit)
                self.pp.update(x, y, yaw, self.path)
                
                # 4. Apply Throttle
                self.car.forward(target_pulse_us)

                # Heartbeat log
                if int(time.time() * 10) % 10 == 0:
                    print(f"Pos: ({x:.2f}, {y:.2f}) | Goal Dist: {dist_to_goal:.2f}m")

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nManual override. Shutting down...")
        finally:
            self.car.shutdown()

"""--- Usage Example ---

square_waypoints = [[0,0], [2,0], [2,2], [0,2], [0,0]]

# Initialize hardware
hw = Control(50, 1500, 1580, 90, 45, 135)

# Setup Navigator (Points every 5cm, lookahead 40cm)
nav = AutonomousNavigator(hw, square_waypoints, wheelbase=0.2, lookahead=0.4, resolution=0.05)

# Start driving!
nav.run(target_pulse_us=1580)
"""