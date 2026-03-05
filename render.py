# laptop_render.py
import zmq
import numpy as np

import pyvista as pv
import numpy as np
import time
import matplotlib.pyplot as plt

def points_to_plane(points, wall_height=2):
    point1 = points[0]
    point2 = points[1]
    lower1 = [point1[0], point1[1], 0]
    lower2 = [point2[0], point2[1], 0]
    upper1 = [point1[0], point1[1], wall_height]
    upper2 = [point2[0], point2[1], wall_height]
    return np.array([lower1, lower2, upper1, upper2])


def extend_points(point1, point2, max_render_distance=5):
    point1 = np.array(point1, dtype=float)
    point2 = np.array(point2, dtype=float)

    direction = point2 - point1
    norm_dir = np.linalg.norm(direction)
    if norm_dir == 0:
        raise ValueError("Points must not be the same")

    direction /= norm_dir  # normalize

    # Quadratic coefficients: ||p + t*d||^2 = R^2
    def solve_t(p, d):
        a = 1  # because d is normalized
        b = 2 * np.dot(p, d)
        c = np.dot(p, p) - max_render_distance ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            return 0, 0
        t1 = (-b + np.sqrt(discriminant)) / 2
        t2 = (-b - np.sqrt(discriminant)) / 2
        return t1, t2

    t1, t2 = solve_t(point1, direction)
    # Use one t for point1 in negative direction, other t for point2 in positive
    new_point1 = point1 + t2 * direction
    new_point2 = point1 + t1 * direction

    return new_point1, new_point2

class CarVisualizer:
    def __init__(self,
                 car_length=0.4,
                 car_width=0.2,
                 base_height=0.1,
                 x_lims=(-10, 10),
                 y_lims=(-10, 10),
                 z_lims=(0, 5)):
        # Hyperparameters
        self.car_length = car_length
        self.car_width = car_width
        self.base_height = base_height
        self.x_lims = x_lims
        self.y_lims = y_lims
        self.z_lims = z_lims

        # Create PyVista plotter
        self.plotter = pv.Plotter()
        self.plotter.set_background("white")
        self.plotter.show_grid()

        # --- Add ground ---
        self._render_ground()

        # --- Add car mesh ---
        self.car_mesh = self._create_car_mesh()
        self.car_actor = self.plotter.add_mesh(self.car_mesh, color="blue")

        # --- Add empty point cloud ---
        self.pc_mesh = self._create_pc_mesh()
        self.pc_actor = self.plotter.add_points(self.pc_mesh, color="red", point_size=5)

        # ---  Add guiding lines mesh ---
        self.gl_meshes = [self._create_line_mesh() for _ in range(6)]
        self.gl_actors = [self.plotter.add_mesh(mesh, color="green", line_width=3) for mesh in self.gl_meshes]

        # --- Add estimated walls mesh ---
        self.wall_mesh1 = self._create_wall_mesh()
        self.wall_actor1 = self.plotter.add_mesh(self.wall_mesh1, color="orange", opacity=0.5)
        self.wall_mesh2 = self._create_wall_mesh()
        self.wall_actor2 = self.plotter.add_mesh(self.wall_mesh2, color="orange", opacity=0.5)

        # Set camera
        self.plotter.camera_position = [( -6, 0, 3), (0,0,0), (0,0,3)]

        # Show plotter window
        self.plotter.show(auto_close=False, interactive_update=True)

    # ------------------ Modular rendering ------------------ #
    def _render_ground(self):
        # Ground plane at z=0
        x = np.linspace(self.x_lims[0], self.x_lims[1], 2)
        y = np.linspace(self.y_lims[0], self.y_lims[1], 2)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        grid = pv.StructuredGrid(X, Y, Z)
        self.plotter.add_mesh(grid, color="lightgray", opacity=0.5)

    def _create_line_mesh(self):
        start = [0, 0, self.base_height]
        end = [0, 0, self.base_height]
        line_mesh = pv.Line(start, end)
        return line_mesh

    def _create_pc_mesh(self):
        """Create an empty point cloud mesh"""
        points = np.zeros((360, 3))
        pc_mesh = pv.PolyData(points)
        return pc_mesh
    def _create_car_mesh(self):
        """Create a triangular prism mesh for the car at origin"""
        l = self.car_length / 2
        w = self.car_width / 2
        h = self.base_height

        # Vertices: front top/bottom, rear left/right top/bottom
        points = np.array([
            [ l, 0, 0],       # front bottom
            [-l,  w, 0],      # rear left bottom
            [-l, -w, 0],      # rear right bottom
            [ l, 0, h],       # front top
            [-l,  w, h],      # rear left top
            [-l, -w, h],      # rear right top
        ])

        # Faces: PyVista needs a special format: [n, p0, p1, ..., pn-1]
        faces = [
            [3, 0,1,2],  # bottom triangle
            [3, 3,5,4],  # top triangle (note flipped for correct normal)
            [4, 0,1,4,3],  # left side
            [4, 0,2,5,3],  # right side
            [4, 1,2,5,4]   # rear face
        ]
        # Flatten faces for PyVista
        faces_flat = np.hstack(faces)
        mesh = pv.PolyData(points, faces_flat)
        return mesh

    def _create_wall_mesh(self):
        point1 = [0, 0]
        point2 = [0, 0]
        points = points_to_plane([point1, point2])
        faces = [4,0,1,3,2]
        faces_flat = np.hstack(faces)
        mesh = pv.PolyData(points, faces_flat)
        return mesh


    # ------------------ Update functions ------------------ #
    def update_pointcloud(self, points):
        assert type(points) == np.ndarray, "Points must be a numpy array"
        assert points.ndim == 2
        """Update LIDAR point cloud"""
        # Ensure points have 3 columns
        if points.shape[1] == 2:
            points_3d = np.hstack([
                points,
                np.full((points.shape[0],1), self.base_height),
            ])
        else:
            points_3d = points

        # mirror along y (side to side dimension)
        points_3d[:, 1] *= -1

        self.pc_mesh.points = points_3d

    def update_guiding_lines(self, endpoints):
        assert type(endpoints) == np.ndarray, "Endpoints must be a numpy array"
        assert endpoints.ndim == 2, "Endpoints must be Nx2 array"
        assert endpoints.shape[0] == 6, "There must be exactly 6 endpoints for the guiding lines"
        assert endpoints.shape[1] == 2, "Endpoints must be 2D (x,y)"

        """Update guiding lines"""
        for i in range(min(len(endpoints), len(self.gl_meshes))):
            start = [0, 0, self.base_height]
            end = [endpoints[i][0], endpoints[i][1], self.base_height]
            self.gl_meshes[i].points = np.array([start, end])

    def update_estimated_walls(self, walls):
        """Placeholder for updating estimated walls (not implemented)"""
        assert type(walls) == np.ndarray, "Walls must be a numpy array"
        assert walls.ndim == 3, "Walls must be a 3D array of shape (2, 2, 2)"
        assert walls.shape[0] == 2, "There must be exactly 2 walls"
        assert walls.shape[1] == 2, "Each wall must have 2 endpoints"
        assert walls.shape[2] == 2, "Endpoints must be 2D (x,y)"
        # Do first wall
        wall1 = walls[0]
        point1 = wall1[0]
        point2 = wall1[1]
        point1, point2 = extend_points(point1, point2)
        points = points_to_plane([point1, point2])
        self.wall_mesh1.points = points

        # Do second wall
        wall2 = walls[1]
        point1 = wall2[0]
        point2 = wall2[1]
        point1, point2 = extend_points(point1, point2)
        points = points_to_plane([point1, point2])
        self.wall_mesh2.points = points

    def update(self, raw_pointcloud_data, guiding_line_endpoints, estimated_walls):
        """Convenience method to update both"""
        if raw_pointcloud_data is not None:
            self.update_pointcloud(raw_pointcloud_data)
        if guiding_line_endpoints is not None:
            self.update_guiding_lines(guiding_line_endpoints)
        if estimated_walls is not None:
            self.update_estimated_walls(estimated_walls)
        self.plotter.update()


if __name__ == "__main__":
    # create renderer
    viz = CarVisualizer()

    # create receiver
    hostname = "b827eb777a0f.dynamic.utexas.edu"
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect(f"tcp://{hostname}:4444")

    # constantly render
    while 1:
        # Receive frame bytes
        try:
            msg = socket.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.error.Again:
            msg = {}

        points = msg.get("points", None)
        endpoints = msg.get("endpoints", None)
        walls = msg.get("walls", None)

        # Update visualization
        viz.update(points, endpoints, walls)

        # if points is not None:
        #     plt.clf()
        #     plt.scatter(points[:, 0], points[:, 1], color="red")
        #     plt.scatter([0],[0], color="blue", s=5)
        #     plt.xlim(-5, 5)
        #     plt.ylim(-5,5)
        #     plt.show(block=False)
        #     plt.pause(0.05)


        # remove from normal loop
        time.sleep(0.03)


