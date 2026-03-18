import zmq

class TcpSocketSender:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.bind("tcp://*:4444")
    
    def send(self, obj: dict):
        """Send any dictionary of data."""
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
        try:
            return self.socket.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.error.Again:
            return {}