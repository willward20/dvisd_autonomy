import numpy as np
from dvisd_autonomy.hardware.lidar import Lidar
from dvisd_autonomy.networking.tcp_socket import TcpSocketSender


class LidarStreamer:
    def __init__(
        self,
        max_range=5.0,
        min_range=0.05,
        fov=None,              # tuple (min_deg, max_deg) or None
        downsample=1,
        channel_name="points"
    ):
        """
        Args:
            max_range: maximum distance to keep (meters)
            min_range: minimum distance to keep (noise filter)
            fov: (min_deg, max_deg) e.g. (-90, 90) for forward view
            downsample: keep every Nth point
            channel_name: key used in TCP message
        """
        self.max_range = max_range
        self.min_range = min_range
        self.fov = fov
        self.downsample = downsample
        self.channel_name = channel_name

        # sensors + networking
        self.lidar = Lidar()
        self.socket = TcpSocketSender()

        # precompute trig
        self.angles = np.arange(360)
        self.radians = np.radians(self.angles)
        self.cos_vals = np.cos(self.radians)
        self.sin_vals = np.sin(self.radians)

        # precompute FOV mask if needed
        if self.fov is not None:
            min_deg, max_deg = self.fov
            self.fov_mask = (self.angles >= min_deg % 360) & (self.angles <= max_deg % 360)
        else:
            self.fov_mask = None

    def process_scan(self, distances):
        """Convert raw scan → filtered XY points"""

        # polar → cartesian
        xs = distances * self.cos_vals
        ys = distances * self.sin_vals
        points = np.stack([xs, ys], axis=1)

        # range filtering
        valid = (distances > self.min_range) & (distances < self.max_range)

        # FOV filtering
        if self.fov_mask is not None:
            valid &= self.fov_mask

        points = points[valid]

        # downsample
        if self.downsample > 1:
            points = points[::self.downsample]

        return points

    def stream(self):
        """Main loop"""
        for distances in self.lidar.get_scans():
            points = self.process_scan(distances)

            self.socket.send({
                self.channel_name: points
            })


if __name__ == "__main__":
    streamer = LidarStreamer()
    streamer.stream()