import threading
from remote import Client, Command
from tasks import Task

class Worker(threading.Thread):
    def __init__(self, tasks, client):
        threading.Thread.__init__(self)

        self.client = client
        self.tasks = tasks

        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill,
            Task.MOVE       : self.task_move,
            Task.SET_AUTO   : self.set_auto
        }

        self.move_time = 0

        self.terminate = False

    def task_connect(self, address):
        self.client.addr = address
        return self.client.connect()

    def task_send(self, msg):
        return self.client.send_cmd_retry(msg)

    def task_kill(self):
        self.terminate = True
        return None

    def task_move(self, keys, schedule_time):
        if self.move_time < schedule_time:
            self.move_time = schedule_time
            vel = int(keys["FORWARD"]) - int(keys["REVERSE"])
            rot = int(keys["RIGHT"]) - int(keys["LEFT"])

            self.client.send_cmd_fmt(Command.SET_VEL, [vel])
            self.client.send_cmd_fmt(Command.SET_ROT, [rot])
        return None
            
    def set_auto(self, auto):
        self.client.send_cmd_fmt(Command.SET_STATE, [auto])
        return None
        
    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                if result:
                    self.tasks.complete(task, result)
