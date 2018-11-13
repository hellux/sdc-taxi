import socket

BUFSIZE = 4096

class Command:
    GET_DATA = 'get_sensor_data'
    GET_MISSION = 'get_mission_status'
    SET_MISSION = 'set_mission'
    SET_MSTATE = 'set_mission_state'
    SET_SPEED = 'set_speed_delta'
    SET_TURN = 'set_turn_delta'
    SET_SPEED_PARAMS = 'set_speed_params'
    SET_TURN_PARAMS = 'set_turn_params'

class Client():
    def __init__(self, addr, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.addr = addr
        self.port = port

    def connect(self):
        self.socket.connect((self.addr, self.port))

    def send(self, string):
        sent = False
        while not sent:
            try:
                self.socket.sendall(string.encode())
                sent = True;
            except BrokenPipeError:
                self.connect()

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
                except Exception as e: print('failed:', e)
                print(response)
            successful = True
