import sys

from remote import Client
from tasks import TaskQueue
from worker import Worker
from gui import GUI

PORT_START = 9000
PORT_END = 9100

def main():
    client = Client(PORT_START, PORT_END)

    if len(sys.argv) > 1:
        inet_addr = sys.argv[1]
        client.addr = inet_addr
        if not client.connect():
            sys.stderr.write('failed to connect to {}\n'.format(inet_addr))

    tasks = TaskQueue()

    wrk = Worker(tasks, client)
    gui = GUI(tasks)

    wrk.start()
    gui.window.mainloop()

if __name__ == '__main__':
    main()
