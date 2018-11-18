from threading import Thread
from remote import Client
import time

PORT_START = 9000
PORT_END = 9100

class Worker(Thread):
    def __init__(self, tasks):
        Thread.__init__(self)
        self.tasks = tasks

    def connect(self):
        self.client = Client("127.0.0.1", PORT_START, PORT_END)
        self.client.connect()

    def run(self):
        self.connect()

        while True:
            task = self.tasks.get()
            print(self.client.send_command(task)[1])
            self.tasks.task_done()
