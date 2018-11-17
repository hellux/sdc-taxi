import socket
import errno
import sys

class Command:
    TEST_CONN    = 'check'
    GET_DATA     = 'get_sensor_data'
    GET_MISSION  = 'get_mission'
    SET_MISSION  = 'set_mission'
    SET_STATE    = 'set_state'
    SET_SPEED    = 'set_speed_delta'
    SET_SPEED_KP = 'set_speed_kp'
    SET_SPEED_KD = 'set_speed_kd'
    SET_SPEED    = 'set_turn_delta'
    SET_SPEED_KP = 'set_turn_kp'
    SET_SPEED_KD = 'set_turn_kd'

class Client():
    BUFSIZE = 4096
    RES_SUCCESS = "success"
    TIMEOUT = 1

    def __init__(self, addr, port_start, port_end):
        self.socket = None
        self.addr = addr
        self.port_start = port_start
        self.port_end = port_end

    def connected(self):
        if self.socket is None: return False
        success, response = self.send_command(Command.TEST_CONN)
        return success

    def connect(self):
        if self.socket: self.socket.close();
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        for port in range(self.port_start, self.port_end+1):
            try:
                self.socket = socket.create_connection(
                    (self.addr, port),
                    Client.TIMEOUT)
                connected = self.connected()
                if connected: break
            except OSError as e:
                if e.errno != errno.ECONNREFUSED:
                    sys.stderr.write("server: {}\n".format(e))
                    return False
        if connected:
            sys.stderr.write("server: connection via port {}\n".format(port))
        return connected

    """
    returns: True if msg sent, otherwise False
    """
    def send(self, string):
        if self.socket is None: return False
        sent = False
        try:
            self.socket.sendall(string.encode())
            sent = True;
        except BrokenPipeError as e:
            sys.stderr.write(e, "\n")
        return sent

    def receive(self):
        return self.socket.recv(Client.BUFSIZE).decode()
        
    """
    returns:
        result: True if command successful, False if command failed, None if
                transmission failed
        response: None if transmission failed, otherwise response from command
    """
    def send_command(self, command, args=[]):
        sent = self.send(':'.join([command, ','.join(map(str, args))]))
        if sent:
            result, response = self.receive().split(':', 1)
            return result == Client.RES_SUCCESS, response
        else:
            return None, None
