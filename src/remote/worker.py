import threading
from remote import Client, Command
from tasks import Task

class Worker(threading.Thread):
    def __init__(self, tasks, client):
        threading.Thread.__init__(self)

        self.client = client
        self.tasks = tasks
        self.counter = 0
        self.counter_dist = 0
        
        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill,
            Task.MOVE       : self.task_move,
            Task.SET_AUTO   : self.set_auto,
            Task.GET_SENSOR : self.get_sensor,
            Task.SET_VEL    : self.set_vel,
            Task.SET_ROT    : self.set_rot,
            Task.GET_MISSION: self.get_mission
        }

        self.move_time = 0

        self.terminate = False

    def send(self, msg):
        print('worker: sending cmd: ', msg)
        return self.client.send_cmd_retry(msg)[1]

    def send_fmt(self, cmd, *args):
        return self.send(Client.create_msg(cmd, *args))

    def task_connect(self, address):
        self.client.addr = address
        return self.client.connect()

    def task_send(self, msg):
        return 'response: {}'.format(self.send(msg))

    def task_kill(self):
        self.terminate = True
        return None
        
    def get_sensor(self):
        self.counter_dist += 10
        if self.counter_dist > 140:
            self.counter_dist = 0
        return ("60, 9, 2," + str(self.counter_dist) + ",0.5").split(',')
        #return self.send_fmt(Command.GET_DATA).split(',')

    def task_move(self, keys, schedule_time):
        if self.move_time < schedule_time:
            self.move_time = schedule_time
            vel = int(keys["FORWARD"]) - int(keys["REVERSE"])
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
        self.counter += 1
        if self.counter > 5:
            self.counter = 0
        return self.counter
        #return self.send_fmt(Command.GET_MISSION)
    
    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                if result:
                    self.tasks.complete(task, result)
