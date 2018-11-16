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
    def __init__(self, addr, port_start, port_end):
        self.socket = None
        self.addr = addr
        self.port_start = port_start
        self.port_end = port_end

    def connect(self):
        if self.socket: self.socket.close();
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        while not connected:
            for port in range(self.port_start, self.port_end+1):
                try:
                    self.socket.connect((self.addr, port))
                    connected = True
                    print("server: connected to port {}".format(port))
                    break
                except Exception as e:
                    pass

    def send(self, string):
        sent = False
        if self.socket is None: self.connect()

        while not sent:
            try:
                self.socket.sendall(string.encode())
                sent = True;
            except BrokenPipeError:
                self.connect()

    def receive(self):
        return self.socket.recv(BUFSIZE).decode()
        
    def send_command(self, command, args=[]):
        successful = False
        self.send(':'.join([command, ','.join(map(str, args))])+';')
        response = self.receive()
        print(response)
