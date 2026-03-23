import numpy as np

def generate_obstacles(N, r_min=0.0, r_max=10.0, obstacle_radius=0.5, seed=None):
    """
    Generate N obstacles randomly in a circular ring around the origin.

    Args:
        N: number of obstacles
        r_min: minimum radius from origin
        r_max: maximum radius from origin
        obstacle_radius: radius of each obstacle
        seed: random seed for reproducibility
    Returns:
        List of (x, y, r) tuples
    """
    rng = np.random.default_rng(seed)
    obstacles = []

    for _ in range(N):
        theta = rng.uniform(0, 2*np.pi)
        r = np.sqrt(rng.uniform(r_min**2, r_max**2))  # uniform in area

        x = r * np.cos(theta)
        y = r * np.sin(theta) 

        obstacles.append((x, y, obstacle_radius))

    return obstacles