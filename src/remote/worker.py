import threading
from remote import Client, Command
from tasks import Task

VEL = 1.3

class Worker(threading.Thread):
    def __init__(self, tasks, client):
        threading.Thread.__init__(self)

        self.client = client
        self.tasks = tasks
        self.counter = 0
        
        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill,
            Task.MOVE       : self.task_move,
            Task.SET_AUTO   : self.set_auto,
            Task.GET_SENSOR : self.get_sensor,
            Task.SET_VEL    : self.set_vel,
            Task.SET_ROT    : self.set_rot,
            Task.GET_MISSION: self.get_mission,
            Task.SEND_MISSION: self.send_mission,
            Task.CLEAR_MISSION: self.clear_mission
        }

        self.move_time = 0

        self.terminate = False

    def send(self, msg):
        if self.client.socket:
            return self.client.send_cmd_retry(msg)[1]
        else:
           return None

    def send_fmt(self, cmd, *args):
        return self.send(Client.create_msg(cmd, *args))

    def task_connect(self, address):
        self.client.addr = address
        return self.client.connect()

    def task_send(self, msg):
        response = self.send(msg)
        return response

    def task_kill(self):
        self.terminate = True
        return None
        
    def get_sensor(self):
        response = self.send_fmt(Command.GET_DATA)
        if response:
            return response.split(' ')
        return None
            
    def task_move(self, keys, schedule_time):
        if self.move_time < schedule_time:
            self.move_time = schedule_time
            vel = VEL*int(keys["FORWARD"]) - VEL*int(keys["REVERSE"])
            rot = int(keys["RIGHT"]) - int(keys["LEFT"])

            self.send_fmt(Command.SET_VEL, vel)
            self.send_fmt(Command.SET_ROT, rot)
        return None
            
    def set_auto(self, auto):
        self.send_fmt(Command.SET_STATE, auto)
        return None

    def set_vel(self, kp, kd):
        if kd:
            self.send_fmt(Command.SET_VEL_KD, float(kd))
        if kp:
            self.send_fmt(Command.SET_VEL_KP, float(kp))   
        return None
    
    def set_rot(self, kp, kd):
        if kd:
             self.send_fmt(Command.SET_ROT_KD, float(kd))
        if kp:
            self.send_fmt(Command.SET_ROT_KP, float(kp))
        return None
    
    def get_mission(self):
        response = self.send_fmt(Command.GET_MISSION)
        if response:
            remaining = response.split(' ')
        else:
            remaining = []

        return len(remaining)

    def send_mission(self, mission):
        self.send_fmt(Command.APPEND_MISSION, *mission)
    
    def clear_mission(self):
        self.send_fmt(Command.SET_MISSION)
        print("MISSION CLEAR")

    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                if result:
                    self.tasks.complete(task, result)
