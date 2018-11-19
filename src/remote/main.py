from gui import GUI
from worker import Worker
from tasks import TaskQueue

def main():
    tasks = TaskQueue()

    wrk = Worker(tasks)
    gui = GUI(tasks)

    wrk.start()
    gui.window.mainloop()

if __name__ == '__main__':
    main()
