from dvisd_autonomy.sensors.lidar import LIDAR
from dvisd_autonomy.render.TcpSocket import TcpSocket
import numpy as np

if __name__ == "__main__":
	# Connect to lidar
	lidar = LIDAR()

	# Create server to send data to laptop
	tcp_socket = TcpSocket()

	# fetch data in a loop
	i = 0
	for distances in lidar.get_data():
		# Convert data to x,y coordinates. 
		angles = np.array([i for i in range(360)])
		radians = np.radians(angles)
		xs = distances * np.cos(radians)
		ys = distances * np.sin(radians)
		points = np.stack([xs,ys], axis=1)
		
		# send to renderer
		print(f"Sending data {i}")
		tcp_socket.send({
			"points": points,
		})
		i += 1

