import threading
from remote import Client
from tasks import Task

PORT_START = 9000
PORT_END = 9100

class Worker(threading.Thread):
    def __init__(self, tasks):
        threading.Thread.__init__(self)

        self.client = None
        self.tasks = tasks

        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill
        }

        self.terminate = False

    def task_connect(self, address):
        self.client = Client(address, PORT_START, PORT_END)
        return self.client.connect()

    def task_send(self, msg):
        if self.client:
            success, response = self.client.send_command(msg)
            return response
        else:
            return "not connected to comm"

    def task_kill(self):
        self.terminate = True

    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                self.tasks.complete(task, result)
