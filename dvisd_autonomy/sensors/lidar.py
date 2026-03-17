import numpy as np
from rplidar import RPLidar, RPLidarException


class Lidar:
    PORT = "/dev/ttyUSB0"
    MAX_RETRIES = 10
    NUM_ANGLES = 360

    def __init__(self, port: str = PORT):
        self.port = port
        self.lidar = None
        self._connect()
        self.scan_data = np.zeros(self.NUM_ANGLES)

    def _connect(self):
        print("Initializing Lidar...")

        for attempt in range(self.MAX_RETRIES):
            try:
                self.lidar = RPLidar(self.port)
                info = self.lidar.get_info()
                print(info)
                print("Lidar Initialized!\n")
                return

            except RPLidarException as e:
                msg = str(e)

                if "Incorrect descriptor" in msg:
                    print(f"LIDAR busy. Restarting... ({attempt})")
                    self._restart()

                elif "could not open port" in msg:
                    raise RuntimeError(
                        f"Could not open port {self.port}. Is the lidar plugged in?"
                    ) from e

                else:
                    raise

        raise RuntimeError("Lidar could not be initialized after retries.")

    def _restart(self):
        if self.lidar is not None:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except Exception:
                pass
            finally:
                self.lidar = None

    def close(self):
        """Explicit cleanup (preferred over __del__)."""
        self._restart()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def get_scans(self):
        """Generator yielding 360-degree scans in meters."""
        while True:
            try:
                self.lidar.clean_input()

                for scan in self.lidar.iter_scans():
                    scan_data = np.zeros(self.NUM_ANGLES)

                    for _, angle, distance_mm in scan:
                        idx = min(self.NUM_ANGLES - 1, int(angle % 360))
                        scan_data[idx] = distance_mm / 1000.0  # mm → m

                    yield scan_data

            except RPLidarException:
                # Recover and continue
                self._restart()
                self._connect()


if __name__ == "__main__":
    # Example usage
    with Lidar() as lidar:
        for i, scan in enumerate(lidar.get_scans()):
            print(scan)
            if i >= 2:
                break