import numpy as np
import matplotlib.pyplot as plt
import time
import os
from dvisd_autonomy.networking.tcp_socket import TcpSocketReceiver


def is_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().lower()
        return "raspberry pi" in model
    except FileNotFoundError:
        return False
    

def load_csv_points(file_name: str = "gt_points.csv"):

    if os.path.exists(file_name):
        loaded_points = np.loadtxt(file_name, delimiter=",", skiprows=1)

        # ensure correct shape
        if loaded_points.ndim == 1:
            loaded_points = loaded_points.reshape(1, -1)

        return loaded_points
    
    return None


# terminate if accidentally running on the car
if is_raspberry_pi():
    raise Exception("You are running this code on the pi, but we can only render on a laptop!")


class LidarTopDownVisualizer:
    def __init__(self, x_lims=(-5, 5), y_lims=(-5, 5)):
        self.x_lims = x_lims
        self.y_lims = y_lims

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        self.ax.set_xlim(*x_lims)
        self.ax.set_ylim(*y_lims)
        self.ax.set_xlabel("X (m)", fontsize=14)
        self.ax.set_ylabel("Y (m)", fontsize=14)
        self.ax.set_aspect('equal')
        self.ax.set_title("Top-Down LIDAR View", fontsize=16)
        self.ax.tick_params(axis='both', labelsize=12)

        # scatter plots
        self.scatter = self.ax.scatter([], [], s=10)

        # draw origin (car position)
        self.ax.plot(0, 0, 'ko')  # black dot at origin

        self.heading = self.ax.quiver(
            0, 0,   # origin
            0.5, 0,   # direction (x=1, y=0)
            angles='xy',
            scale_units='xy',
            scale=0.75,
            color='r',
            width=0.0075
        )

        gt_points = load_csv_points()
        if gt_points is not None:
            gt_points = np.asarray(gt_points)

            # ensure Nx2
            if gt_points.shape[1] > 2:
                gt_points = gt_points[:, :2]

            # plot GT points (different color + smaller size)
            self.gt_scatter = self.ax.scatter(
                gt_points[:, 0],
                gt_points[:, 1],
                s=5,
                c='g',
                alpha=0.5,
                label="GT Map"
            )
        

        plt.grid()

        plt.show()

    def update_points(self, points, scatter):
        if points is None or len(points) == 0:
            scatter.set_offsets(np.empty((0, 2)))
            return

        points = np.asarray(points)

        # ensure Nx2
        if points.shape[1] > 2:
            points = points[:, :2]

        # mirror y (keep your original convention)
        points[:, 1] *= -1

        scatter.set_offsets(points)

        return points

    def update(self, points=None):
        points = self.update_points(points, self.scatter)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        # viz.save_figure(points)
        # exit()


    def save_figure(self, points, filename="gt_plot.png"):
        # Save image
        self.fig.savefig(
            filename,
            dpi=600,
            bbox_inches='tight'
        )

        # Save CSV (same name, different extension)
        csv_filename = "gt_points.csv"

        if points is not None and len(points) > 0:
            points = np.asarray(points)

            # ensure Nx2
            if points.shape[1] > 2:
                points = points[:, :2]

            np.savetxt(csv_filename, points, delimiter=",", header="x,y", comments="")
            print(f"Saved figure to {filename} and points to {csv_filename}")
        else:
            print(f"Saved figure to {filename} (no points to save)")


hostname_map = {
    1: {
        "ut": "b827eb777a0f.dynamic.utexas.edu",
        "radio": "192.168.168.11"
    },
    2: {
        "ut": "b827eb2f3fa5.dynamic.utexas.edu",
        "radio": "192.168.168.12"
    },
    3: {
        "ut": "b827ebfbf87a.dynamic.utexas.edu",
        "radio": "192.168.168.13"
    },
}


if __name__ == "__main__":
    car = 1
    wifi_type = "radio"
    assert wifi_type in ["ut", "radio"]

    viz = LidarTopDownVisualizer()

    hostname = hostname_map[car][wifi_type]
    receiver = TcpSocketReceiver(hostname)

    while True:
        msg = receiver.receive()

        if len(msg) > 0:
            viz.update(
                points=msg.get("points"),
            )

        time.sleep(0.03)