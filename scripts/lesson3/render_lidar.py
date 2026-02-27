import cv2
import numpy as np
from rplidar import RPLidar, RPLidarException

WINDOW_SIZE = 800
MAX_DISTANCE = 3000  # Max distance to display in mm (3000mm = 3m)

image = np.zeros((WINDOW_SIZE, WINDOW_SIZE, 3), dtype=np.uint8)
scan_data = np.zeros(360)

def update_view(image, scan_data):
    """Refreshes the image with the latest scan data"""
    image[:] = 0 # Clear screen to black

    center_x = WINDOW_SIZE // 2
    center_y = WINDOW_SIZE // 2
    scale = min(center_x, center_y) / MAX_DISTANCE

    # Draw the center reference point (robot location)
    cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)

    for angle in range(360):
        color = (0, 255, 0)
        if angle < 20 or angle > 340:
            color = (255, 0, 0)
        distance = scan_data[angle]
        if distance > 0:
            radians = np.radians(angle)
            
            x = distance * np.cos(radians)
            y = distance * np.sin(radians)

            # Map to screen coordinates
            # +center to move (0,0) to middle of screen
            px = int(center_x + x * scale)
            py = int(center_y + y * scale)

            # Draw point (Green)
            if 0 <= px < WINDOW_SIZE and 0 <= py < WINDOW_SIZE:
                cv2.circle(image, (px, py), 2, color, -1)

    cv2.imshow('Lidar Scan', image)

retry = 0
while retry < 10:
	try:
		lidar = RPLidar("/dev/ttyUSB0")
		info = lidar.get_info()
		for scan in lidar.iter_scans():
			scan_data[:] = 0
			for (_, angle, distance) in scan:
				scan_angle = min(359, int(angle % 360))
				scan_data[scan_angle] = distance
			update_view(image, scan_data)
			key = cv2.waitKey(1) & 0xFF
			if key == ord('q'):
				retry = 10
				break
	except Exception as e:
		print(f"Retry {retry}:", e)
		lidar.stop()
		lidar.disconnect()
		retry += 1
