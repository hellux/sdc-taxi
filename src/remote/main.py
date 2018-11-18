CONN_PORT_START = 9000
CONN_PORT_END = 9100

from gui import GUI
from remote import Client
from worker import Worker

def main():
    # TODO create common queue for communication between worker and gui

    wrk = Worker()
    gui = GUI()

    gui.window.mainloop()

if __name__ == '__main__':
    main()
