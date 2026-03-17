import numpy as np
import matplotlib.pyplot as plt

DT = 0.05
L = 0.3
MAX_RANGE = 5.0

N_LIDAR = 180
LIDAR_ANGLES = np.linspace(-np.pi/4, np.pi/4, int(N_LIDAR/4))

SPEED = 1.0

SERVO_CENTER = 100
SERVO_MIN = 50
SERVO_MAX = 140

MAX_STEER_RAD = 0.5  # same as your current clip (~28.6 deg)

def steering_to_servo(delta):
    """
    Convert steering angle (radians) → servo command
    """
    normalized = np.clip(delta / MAX_STEER_RAD, -1.0, 1.0)
    cmd = SERVO_CENTER + normalized * (SERVO_MAX - SERVO_CENTER)
    return np.clip(cmd, SERVO_MIN, SERVO_MAX)


def servo_to_steering(cmd):
    """
    Convert servo command → steering angle (radians)
    """
    normalized = (cmd - SERVO_CENTER) / (SERVO_MAX - SERVO_CENTER)
    return normalized * MAX_STEER_RAD


def generate_obstacles(N, r_min=0.0, r_max=10.0, obstacle_radius=0.4, seed=None):
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







def simulate_lidar_fast(state, obstacles, max_range=MAX_RANGE, n_points=200):
    """
    Fast LiDAR simulation using vectorization.
    """
    x, y, heading = state
    ranges = np.full(len(LIDAR_ANGLES), max_range)

    # Precompute points along rays
    ds = np.linspace(0, max_range, n_points)
    rays_x = np.cos(LIDAR_ANGLES[:, None] + heading) * ds  # shape: [n_beams, n_points]
    rays_y = np.sin(LIDAR_ANGLES[:, None] + heading) * ds

    # Shift to robot position
    rays_x += x
    rays_y += y

    # Convert obstacles to arrays
    obs = np.array(obstacles)  # shape [n_obstacles, 3]
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


def best_direction(ranges):

    v = 2.0 * np.array([1.0, 0.0])

    for r, theta in zip(ranges, LIDAR_ANGLES):

        if r > 3.0:
            continue

        r = max(r, 0.2)

        w = 1.0 / (r*r)

        vx = -w * np.cos(theta)
        vy = -w * np.sin(theta)

        v += np.array([vx, vy])

    angle = np.arctan2(v[1], v[0])

    return angle


def bicycle_step(state, delta):

    x, y, psi = state

    x += SPEED * np.cos(psi) * DT
    y += SPEED * np.sin(psi) * DT
    psi += SPEED/L * np.tan(delta) * DT

    return np.array([x, y, psi])



x = -2.5
y = -2.5
psi = np.pi/4
state = np.array([x, y, psi])  # x, y, heading
trajectory = []
obstacles = generate_obstacles(N=120)


while (np.sqrt(state[0]**2 + state[1]**2) < 10.0):

    ranges = simulate_lidar_fast(state, obstacles)

    desired_angle = best_direction(ranges)

    # Controller outputs desired steering angle
    delta_desired = np.clip(desired_angle, -MAX_STEER_RAD, MAX_STEER_RAD)

    # Convert to real robot command
    servo_cmd = steering_to_servo(delta_desired)
    print("servo_cmd: ", servo_cmd)

    # Convert back to actual steering (models actuator limits)
    delta_actual = servo_to_steering(servo_cmd)
    # print("delta: ", delta_actual)

    state = bicycle_step(state, delta_actual)

    trajectory.append(state.copy())


trajectory = np.array(trajectory)




fig, ax = plt.subplots(figsize=(8, 8))

# Plot obstacles
for ox, oy, r in obstacles:
    circle = plt.Circle((ox, oy), r*0.75, color='gray')#, alpha=0.5)
    ax.add_patch(circle)

# Dashed circle at radius 10
circle_outer = plt.Circle((0,0), 10, color='black', linestyle='--', fill=False)#, alpha=0.7)
ax.add_patch(circle_outer)

# Dashed circle at radius 4.0
circle_inner = plt.Circle((0,0), 4.0, color='black', linestyle='--', fill=False)#, alpha=0.7)
ax.add_patch(circle_inner)

# Yellow stars around radius 11 every 36 degrees
star_radius = 11
angles = np.deg2rad(np.arange(0, 360, 36))
star_x = star_radius * np.cos(angles)
star_y = star_radius * np.sin(angles)
ax.scatter(star_x, star_y, marker='*', s=300, color='green', label="Prizes")

ax.plot(trajectory[:,0], trajectory[:,1], linewidth=2.0, label='Path')

# Plot robot initial position
ax.plot(x, y, 'bo', markersize=12, label="Robot Start")

# Plot robot heading as an arrow
arrow_length = 1.0
ax.arrow(x, y, arrow_length*np.cos(psi), arrow_length*np.sin(psi),
         head_width=0.4, head_length=0.4, fc='blue', ec='blue')

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Obstacle Course and Robot Start Pose")
ax.set_aspect('equal')
ax.grid(True)
ax.legend()

# plt.axis("equal")
# plt.savefig("example.png")
plt.show()