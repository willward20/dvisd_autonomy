import zmq


class TcpSocket:
	def __init__(self):
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.PUSH)
		self.socket.bind("tcp://*:4444")
	
	def send(self, obj):
		try:
			self.socket.send_pyobj(obj, flags=zmq.NOBLOCK)
			return True
		except zmq.error.Again:
			return False
