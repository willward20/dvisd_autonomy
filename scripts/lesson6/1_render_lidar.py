from dvisd_autonomy.sensors.lidar import LIDAR
from dvisd_autonomy.render.TcpSocket import TcpSocket
import numpy as np
from tqdm import tqdm


#######################################################
# Part 1: Render Lidar
# You have already done this step in a previous lesson, 
# but the rendering script is slightly different now. 
# You will run this script on the rasberry pi (car)
# and it will broadcast the data. 
# Then a separate script running on the laptop will listen to this data, 
# and plot the point cloud. 
# eventually this will allow us to render what the car sees while its driving. 
# For now, just get this script working and then move on to script 2. 
#######################################################


if __name__ == "__main__":
	# Connect to lidar
	lidar = LIDAR()

	# Create server to send data to laptop
	tcp_socket = TcpSocket()

	# fetch data in a loop
	# NOTE: tqdm creates the progress bar. 
	for distances in tqdm(lidar.get_data(), desc="Fetching LIDAR data"):
		# Convert data to x,y coordinates. 
		angles = np.array([i for i in range(360)])
		radians = np.radians(angles)
		xs = distances * np.cos(radians)
		ys = distances * np.sin(radians)
		points = np.stack([xs,ys], axis=1)
		
		# send to renderer
		tcp_socket.send({
			"points_red": points,
		})

