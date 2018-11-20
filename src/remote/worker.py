import threading
from remote import Client
from tasks import Task

class Worker(threading.Thread):
    SPEED_FORWARD = 0.078
    SPEED_REVERSE = 0.071
    RIGHT_TURN = 0.1
    LEFT_TURN = 0.05
    SPEED_STOP = 0.075
    
    def __init__(self, tasks, client):
        threading.Thread.__init__(self)

        self.client = client
        self.tasks = tasks

        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill,
            Task.MOVE       : self.task_move
        }

        self.terminate = False

    def task_connect(self, address):
        self.client.addr = address
        return self.client.connect()

    def task_send(self, msg):
        return self.client.send_cmd_retry(msg)

    def task_kill(self):
        self.terminate = True

    def task_move(self, keys):
        if(keys["LEFT"]):
            task_send(self, SET_SPEED)
            task_send(self, SPEED_FORWARD)
            task_send(self, SET_ROT)
            task_send(self, LEFT_TURN)
            keys["LEFT"] = False
            
        if(keys["RIGHT"]):
            task_send(self, SET_SPEED)
            task_send(self, SPEED_FORWARD)
            task_send(self, SET_ROT)
            task_send(self, RIGHT_TURN)
            keys["RIGHT"] = False
            
        if(keys["FORWARD"]):
            task_send(self, SET_SPEED)
            task_send(self, SPEED_FORWARD)
            keys["FORWARD"] = False
            
        if(keys["REVERSE"]):
            task_send(self, SET_SPEED)
            task_send(self, SPEED_REVERSE)
            keys["REVERSE"] = False
            
        if(all(dir == False for dir in keys.values())):
            task_send(self, SET_SPEED)
            task_send(self, SPEED_STOP)
            
    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                self.tasks.complete(task, result)
