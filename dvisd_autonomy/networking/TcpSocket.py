import zmq

acceptable_data_types = [
	"points_red",
	"points_blue",
	"points_green",
	"endpoints",
	"walls",
	"waypoint",
	"path",
]

class TcpSocketSender:
	def __init__(self):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.PUSH)
		self.socket.bind("tcp://*:4444")
	
	def send(self, obj:dict):
		# check correct keys in dict
		for k in obj.keys():
			if k not in acceptable_data_types:
				raise ValueError(f"Key {k} not in acceptable data types: {acceptable_data_types}")

		# send
		try:
			self.socket.send_pyobj(obj, flags=zmq.NOBLOCK)
			return True
		except zmq.error.Again:
			return False

class TcpSocketReceiver:
	def __init__(self, hostname):
			self.context = zmq.Context()
			self.socket = self.context.socket(zmq.PULL)
			self.socket.connect(f"tcp://{hostname}:4444")
	
	def receive(self) -> dict:
		# This function checks if there is a message to receive, and if so, receives it. 
		# Otherwise, it returns an empty dict.
		try:
			msg = socket.recv_pyobj(flags=zmq.NOBLOCK)
		except zmq.error.Again:
			msg = {}

		# check correct keys
		for k in msg.keys():
			if k not in acceptable_data_types:
				raise ValueError(f"Key {k} not in acceptable data types: {acceptable_data_types}")
		return msg