from enum import Enum, auto
import queue

class Task(Enum):
    CONNECT = auto()
    SEND = auto()
    KILL = auto()
    MOVE = auto()
    SET_AUTO = auto()
    GET_SENSOR = auto()
    SET_VEL = auto()
    SET_ROT = auto()
    GET_MISSION = auto()
    SEND_MISSION = auto()
    CLEAR_MISSION = auto()

class TaskQueue():
    def __init__(self):
        self.tasks = queue.LifoQueue()
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
