import threading
from remote import Client
from tasks import Task

class Worker(threading.Thread):
    def __init__(self, tasks, client):
        threading.Thread.__init__(self)

        self.client = client
        self.tasks = tasks

        self.actions = {
            Task.CONNECT    : self.task_connect,
            Task.SEND       : self.task_send,
            Task.KILL       : self.task_kill
        }

        self.terminate = False

    def task_connect(self, address):
        self.client.addr = address
        return self.client.connect()

    def task_send(self, msg):
        return self.client.send_cmd_retry(msg)

    def task_kill(self):
        self.terminate = True

    def run(self):
        while not self.terminate:
            task, args = self.tasks.get(block=True)
            action = self.actions.get(task)
            if action:
                result = action(*args)
                self.tasks.complete(task, result)
