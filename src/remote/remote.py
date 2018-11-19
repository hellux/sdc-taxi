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
    CMD_SEP = ':'
    ARG_SEP = ','

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
        if self.socket:
            sent = False
            try:
                self.socket.sendall(string.encode())
                sent = True;
            except BrokenPipeError as e:
                sys.stderr.write("{}\n".format(e))
            return sent
        else:
            return False

    def receive(self):
        if self.socket:
            return self.socket.recv(Client.BUFSIZE).decode()
        else:
            return None

    def clear(self):
        if self.socket:
            timeout = self.socket.gettimeout()
            self.socket.setblocking(False)
            try: self.receive()
            except OSError: pass
            self.socket.settimeout(timeout)
        
    """
    returns:
        result: True if command successful, False if command failed, None if
                transmission failed
        response: None if transmission failed, otherwise response from command
    """
    def send_command(self, msg):
        self.clear()
        sent = self.send(msg)
        if sent:
            msg = self.receive()
            if msg.find(Client.CMD_SEP) >= 0:
                result, response = msg.split(Client.CMD_SEP, 1)
                return result == Client.RES_SUCCESS, response

        return None, None

    def send_command_fmt(self, command, args=[]):
        msg = command + Client.CMD_SEP + Client.ARG_SEP.join(map(str, args))
        return self.send_command(msg)
