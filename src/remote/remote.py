import socket
import errno
import sys

class Command:
    TEST_CONN       = 'check'
    GET_DATA        = 'get_sensor'
    GET_MISSION     = 'get_miss'
    SET_MISSION     = 'set_miss'
    APPEND_MISSION  = 'app_miss'
    SET_STATE       = 'set_auto'
    SET_VEL         = 'set_vel'
    SET_ROT         = 'set_rot'
    SET_VEL_KP      = 'set_vel_kp'
    SET_VEL_KD      = 'set_vel_kd'
    SET_ROT_KP      = 'set_rot_kp'
    SET_ROT_KD      = 'set_rot_kd'
    SHUTDOWN        = 'shutdown'

class Client():
    BUFSIZE = 4096
    RES_SUCCESS = "success"
    TIMEOUT = 1
    CMD_SEP = ':'
    ARG_SEP = ','

    def __init__(self, port_start, port_end):
        self.socket = None
        self.port_start = port_start
        self.port_end = port_end

    def connected(self):
        if self.socket is None: return False
        success, response = self.send_cmd(Command.TEST_CONN)
        return success == True

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

    def receive(self):
        if self.socket:
            return self.socket.recv(Client.BUFSIZE).decode()
        else:
            return None

    def clear_socket(self):
        if self.socket:
            timeout = self.socket.gettimeout()
            self.socket.setblocking(False)
            try: self.receive()
            except OSError: pass
            self.socket.settimeout(timeout)
    
    """
    returns:
        result: True if cmd successful, False if cmd failed, None if
                transmission failed
        response: None if transmission failed, otherwise response from cmd
    """
    def send_cmd(self, msg):
        self.clear_socket()
        sent = False
        try:
            self.socket.sendall(msg.encode())
            sent = True;
        except BrokenPipeError as e:
            sys.stderr.write("{}\n".format(e))
        if sent:
            try: msg = self.receive()
            except: return None, None
            if msg and msg.find(Client.CMD_SEP) >= 0:
                result, response = msg.split(Client.CMD_SEP, 1)
                return result == Client.RES_SUCCESS, response
            else:
                sys.stderr.write(
                    'server: invalid response received -- "{}"\n'.format(msg))
        return None, None

    def send_cmd_retry(self, msg):
        succ, resp = self.send_cmd(msg)
        if succ is None:
            if self.connect():
                succ, resp = self.send_cmd(msg)
        return succ, resp

    @staticmethod
    def create_msg(cmd, *args):
        return cmd + Client.CMD_SEP + Client.ARG_SEP.join(map(str, args))
