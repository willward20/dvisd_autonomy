import numpy as np
import matplotlib.pyplot as plt
from dvisd_autonomy.simulation.bicycle_model import BicycleModel
from dvisd_autonomy.simulation.lidar import LidarModel
from dvisd_autonomy.simulation.obstacles import generate_obstacles


N_LIDAR = 180
LIDAR_ANGLES = np.linspace(-np.pi/4, np.pi/4, int(N_LIDAR/4))


def best_direction(ranges):

    v = 0.5 * np.array([1.0, 0.0])

    for r, theta in zip(ranges, LIDAR_ANGLES):

        if r > 3.0:
            continue

        r = max(r, 0.2)

        x = np.cos(theta)
        y = np.sin(theta)

        w = 1.0 / (r*r)
     
        if y >= 0:
            vx = w * y
            vy = - w * x
        else:
            vx = - w * y
            vy = w * x


        v += np.array([vx, vy])

    angle = np.arctan2(v[1], v[0])

    return angle


def find_best_gap(ranges, min_dist=1.0):
    """
    Find largest contiguous region where range > min_dist
    """
    valid = ranges > min_dist

    gaps = []
    start = None

    for i, v in enumerate(valid):
        if v and start is None:
            start = i
        elif not v and start is not None:
            gaps.append((start, i))
            start = None

    if start is not None:
        gaps.append((start, len(ranges)))

    if not gaps:
        return None

    return max(gaps, key=lambda g: g[1] - g[0])


def select_goal_point(ranges, angles, gap):
    """
    Pick midpoint of gap as goal
    """
    start, end = gap
    mid = (start + end) // 2

    r = ranges[mid]
    theta = angles[mid]

    x = r * np.cos(theta)
    y = r * np.sin(theta)

    return np.array([x, y])


def pure_pursuit_angle(goal_point):
    """
    Steering angle toward goal point (robot frame)
    """
    return np.arctan2(goal_point[1], goal_point[0])




# Create a simulation
bicycle = BicycleModel()
lidar = LidarModel(n_beams=N_LIDAR)
obstacles = generate_obstacles(N=40)

# Set initial state
x = -2.5
y = -2.5
psi = np.pi/4
state = np.array([x, y, psi])
trajectory = []


while (np.sqrt(state[0]**2 + state[1]**2) < 10.0):

    ranges = lidar.simulate(state, obstacles)


    # ----- This one uses force vector field -----
    desired_angle = best_direction(ranges)
    # --------------------------------------------

    # ---- This one uses largest gap pursuit -----
    # gap = find_best_gap(ranges)

    # if gap is not None:
    #     goal = select_goal_point(ranges, LIDAR_ANGLES, gap)
    #     desired_angle = pure_pursuit_angle(goal)
    # else:
    #     desired_angle = 0.0  # fallback
    # --------------------------------------------

    state = bicycle.step(state, desired_angle)

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