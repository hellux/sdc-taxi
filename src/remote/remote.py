import socket

BUFSIZE = 4096

class Command:
    GET_DATA = 'get_sensor_data'
    GET_MISSION = 'get_mission_status'
    SET_MISSION = 'set_mission'
    EXECUTE = 'set_mission'
    CANCEL = 'cancel_mission'
    SET_SPEED = 'set_speed_delta'
    SET_TURN = 'set_turn_delta'
    SET_SPEED_PARAMS = 'set_speed_params'
    SET_TURN_PARAMS = 'set_turn_params'

class Client():
    def __init__(self, addr, port):
        self.socket = None
        self.addr = addr
        self.port = port

    def connect(self):
        if self.socket: self.socket.close();
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.addr, self.port))

    def send(self, string):
        sent = False
        if self.socket is None: self.connect()

        while not sent:
            try:
                self.socket.sendall(string.encode())
                sent = True;
            except BrokenPipeError:
                self.socket.connect()

    def receive(self):
        while True:
            return self.socket.recv(BUFSIZE).decode()
        
    def send_command(self, command, args=[]):
        successful = False
        while not successful:
            self.send(':'.join([command, ','.join(map(str, args))])+';')
            if command[0:3] == "get":
                response = ''
                try: response = self.receive()
                except e: print('failed:', e)
                print(response)
            successful = True
