import socket

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
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((addr, port))
        
    def send_command(self, command, args=[]):
        data = ":".join([command, ','.join(map(str, args))])
        #self.socket.sendall(data.encode())
        for i in range(10):
            self.socket.sendall("din wpm: {}".format(i).encode())
