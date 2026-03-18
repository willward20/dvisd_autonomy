# Networking

`tcp_socket.py` provides an interface for sending data from the raspberry pi to other computers via the internet. 

Run `lidar_streamer.py` on the raspberry pi to transmit LiDAR point data.

Run `lidar_render.py` on a different computer to visualize LiDAR points. Note that you can't run this script on the Pi because it's too small to render the code.