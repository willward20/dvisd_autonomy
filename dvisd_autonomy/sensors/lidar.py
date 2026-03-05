import numpy as np
import time
from rplidar import RPLidar

class LIDAR: 
    """Sensor interface for  LiDAR"""

    def __init__(self):
        self.lidar = RPLidar("/dev/ttyUSB0")

    def __del__(self):
        """Cleans up the LiDAR sensor."""
        self.lidar.stop()
        self.lidar.disconnect()

    def get_scans(self):
        return self.lidar.iter_scans()