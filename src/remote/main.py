from gui import GUI
from worker import Worker
from queue import Queue

def main():
    worker_tasks = Queue()

    wrk = Worker(worker_tasks)
    gui = GUI(worker_tasks)

    wrk.start()
    gui.window.mainloop()
    # TODO döda gui vid quit
    wrk.stop()

if __name__ == '__main__':
    main()
