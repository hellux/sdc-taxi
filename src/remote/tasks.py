from enum import Enum, auto
import queue

class Task(Enum):
    CONNECT = auto()
    SEND = auto()
    KILL = auto()

class TaskQueue():
    def __init__(self):
        self.tasks = queue.Queue()
        self.completed = queue.Queue()

    """
    methods for putr thread
    """

    def put(self, task, *args):
        if not type(task) is Task:
            raise ValueError('wrong type for task')
        self.tasks.put((task, args))

    def get_completed(self, block=False):
        try:
            return self.completed.get(block)
        except queue.Empty:
            return None

    """
    methods for worker thread
    """

    def get(self, block=True):
        return self.tasks.get(block)

    def complete(self, task, *result):
        if not type(task) is Task:
            raise ValueError('wrong type for task')
        self.completed.put((task, result))
