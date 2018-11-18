from threading import Thread
import time; 

class Worker(Thread):

    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while True:
            print("hej från tråd")
            time.sleep(5)
        

