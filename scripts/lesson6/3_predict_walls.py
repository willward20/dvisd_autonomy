from dvisd_autonomy.sensors.lidar import LIDAR
from dvisd_autonomy.render.TcpSocket import TcpSocket
import numpy as np
from tqdm import tqdm


#######################################################
# Part 3: Predict the walls!
# In the previous script, we highlighted points that belong to each wall
# Now we will compute a line of best fit, and draw the actual wall itself
# we assume the wall is flat. Is this a good assumption?
#######################################################


def line_of_best_fit(points):

	# Remove points close to 0,0
	margin_of_error = 0.05 # 5 cm
	zero_points = (np.abs(points[:, 0]) < margin_of_error) & (np.abs(points[:, 1]) < margin_of_error)
	points = points[~zero_points]

	if points.shape[0] <= 1:
		return np.array([[0.0, 0.0], [0.0, 0.0]])

	# Removes points that are far from the middle of the data
	# This is necesarry because the lidar is noisy, and sometimes 
	# it hallucinates points that are not there.
	# our line of best fit is very sensitive to this noise
	median = np.median(points, axis=0) # compute middle points
	dists = np.linalg.norm(points - median, axis=1) # compute distances
	keep = dists < np.percentile(dists, 80) # remove points that are in the 80th percentile of distance or higher
	points = points[keep]

	if points.shape[0] <= 1:
		return np.array([[0.0, 0.0], [0.0, 0.0]])

	#  pull out x and y
	x = points[:, 0]
	y = points[:, 1]

	# fit y = mx + b
	m, b = np.polyfit(y, x, 1)

	# Find two points along the line
	y1, y2 = y.min(), y.max()
	x1 = m * y1 + b
	x2 = m * y2 + b

	# return two points along the wall. 
	p1 = [x1, y1]
	p2 = [x2, y2]
	return np.array([p1, p2])



if __name__ == "__main__":
	# How wide of a range of points do you want to use to predict the wall?
	angle = 25 # degrees

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

		# Now we will compute the line of best fit for each wall, and draw that as well.
		# This function first calculates the line that goes through the data
		# then it returns two points along this line for plotting. 
		left_wall = line_of_best_fit(left_wall_points)
		right_wall = line_of_best_fit(right_wall_points)
		walls = np.stack([left_wall, right_wall])

		# send to renderer, using different colors for each wall
		tcp_socket.send({
			"points_red": other_points,
			"points_blue": left_wall_points,
			"points_green": right_wall_points,
			"endpoints": endpoints,
			"walls": walls,
		})
		