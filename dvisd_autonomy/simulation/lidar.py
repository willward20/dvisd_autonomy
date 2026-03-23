import numpy as np

class LidarModel:
    def __init__(self, n_beams=180, fov=np.pi/2, max_range=5.0, n_points=200):
        """
        LiDAR simulator using vectorized ray casting.

        Args:
            n_beams: number of LiDAR beams
            fov: field of view in radians
            max_range: maximum LiDAR range
            n_points: number of points along each ray
        """
        self.n_beams = n_beams
        self.fov = fov
        self.max_range = max_range
        self.n_points = n_points
        self.angles = np.linspace(-fov/2, fov/2, int(n_beams/4))


    def simulate(self, state, obstacles):
        """
        Simulate LiDAR scan.

        Args:
            state: [x, y, heading] of robot
            obstacles: list of (x, y, r) tuples

        Returns:
            ranges: np.array of length n_beams
        """
        x, y, heading = state
        ranges = np.full(self.n_beams, self.max_range)

        # Precompute points along rays
        ds = np.linspace(0, self.max_range, self.n_points)
        rays_x = np.cos(self.angles[:, None] + heading) * ds  # [n_beams, n_points]
        rays_y = np.sin(self.angles[:, None] + heading) * ds
        rays_x += x
        rays_y += y

        if len(obstacles) == 0:
            return ranges

        # Convert obstacles to arrays
        obs = np.array(obstacles)  # [n_obs, 3]
        ox = obs[:, 0][:, None, None]  # [n_obs,1,1]
        oy = obs[:, 1][:, None, None]
        r = obs[:, 2][:, None, None]

        # Compute squared distance from every ray point to every obstacle
        dx = rays_x[None, :, :] - ox  # [n_obs, n_beams, n_points]
        dy = rays_y[None, :, :] - oy
        dist2 = dx**2 + dy**2

        # Find where distance < obstacle radius^2
        hits = dist2 < r**2

        # For each beam, find first obstacle hit
        for i in range(hits.shape[1]):  # beam index
            hit_points = np.where(hits[:, i, :])[1]
            if len(hit_points) > 0:
                ranges[i] = ds[hit_points[0]]

        return ranges