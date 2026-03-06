from dvisd_autonomy.sensors.lidar import LIDAR
from dvisd_autonomy.render.TcpSocket import TcpSocket
import numpy as np
from tqdm import tqdm


#######################################################
# Part 2: Find the walls!
# In this script, we will highlight the walls to the left and right of the car
# To do so, we will specify a variable called "angle".
# then, for any points that are in the range of 90 degrees (left) or 270 degrees (right) plus or minus "angle", we will assume they belong to the wall.
# For example, if angle is 45 degrees, then any points between 45 and
# 135 degrees will be assumed to belong to the left wall, 
# and any points between 225 and 315 degrees will be assumed to belong to the right wall.
# You can change the "angle" variable to see how it affects the points that are classified
#######################################################


if __name__ == "__main__":
	# How wide of a range of points do you want to use to predict the wall?
	angle = 45 # degrees

	# Connect to lidar
	lidar = LIDAR()

	# Create server to send data to laptop
	# TCP is the communication protocol that most of the internet uses. 
	tcp_socket = TcpSocket()

	# fetch data in a loop
	# Press CTRL+C to stop the program
	for distances in tqdm(lidar.get_data(), desc="Fetching LIDAR data"):
		# Convert data to x,y coordinates. 
		angles = np.array([i for i in range(360)])
		radians = np.radians(angles)
		xs = distances * np.cos(radians)
		ys = distances * np.sin(radians)
		points = np.stack([xs,ys], axis=1)

		# Compute points that we assume belong to the left wall.
		# Remember 0 degrees is forward. 90 degrees is perfectly to the left.
		# Therefore, we assume 90 degrees plus or minus "angle" must belong to the wall
		front_left_angle = 90 - angle
		back_left_angle = 90 + angle
		belongs_to_left_wall = (angles > front_left_angle) & (angles < back_left_angle)

		# do the same thing for the right wall
		# right = 270 degrees
		front_right_angle = 270 + angle
		back_right_angle = 270 - angle
		belongs_to_right_wall = (angles > back_right_angle) & (angles < front_right_angle)

		# now create points that belong to each list
		# This assigns the point to left_wall_points if it belongs to the left wall, otherwise assigns it to 0.0
		left_wall_points = points[belongs_to_left_wall]
		
		# do the same thing for the right wall
		right_wall_points = points[belongs_to_right_wall]
		
		# now gather any points that do not belong to either wall.
		# Thats what this means: ~(belongs_to_left_wall | belongs_to_right_wall)
		# ~ means "NOT", and | means "OR"
		# so you can read this as:
		# NOT belongs to left wall OR belongs to right wall
		other_points = points[~(belongs_to_left_wall | belongs_to_right_wall)]


		# We will also draw lines from the car (origin) at the specified angle,
		# so that its easier to see whats happening
		# Feel free to change the "angle" variable to get an idea of whats happening. 
		front_left_endpoint = [np.cos(np.radians(front_left_angle)), np.sin(np.radians(front_left_angle))]
		back_left_endpoint = [np.cos(np.radians(back_left_angle)), np.sin(np.radians(back_left_angle))]
		front_right_endpoint = [np.cos(np.radians(front_right_angle)), np.sin(np.radians(front_right_angle))]
		back_right_endpoint = [np.cos(np.radians(back_right_angle)), np.sin(np.radians(back_right_angle))]

		# combine into a single array
		endpoints = np.array([
			front_left_endpoint, 
			back_left_endpoint, 
			front_right_endpoint, 
			back_right_endpoint
		])
		endpoints *= 10 # move the endpoint out to 10 meters so the line is longer. 


		# send to renderer, using different colors for each wall
		tcp_socket.send({
			"points_red": other_points,
			"points_blue": left_wall_points,
			"points_green": right_wall_points,
			"endpoints": endpoints,
		})
		