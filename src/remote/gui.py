import tkinter as tk
from course import Node, NodeType
from tasks import Task

class GUI():
    LOOP_DELAY = 50

    def __init__(self, tasks):
        self.tasks = tasks

        self.complete_actions = {
            Task.CONNECT    : print,
            Task.SEND       : print,
        }

        self.window = tk.Tk()
        self.init_gui()
        self.window.protocol('WM_DELETE_WINDOW', self.quit)
        self.window.after(2000, self.main_loop)

    def init_gui(self):
        #VARIABLES
        self.carSpeed = 25
        self.drivingMode = "Auto"

        #self.window COMPONENTS
        infoFrame = tk.Frame(self.window, highlightbackground="red")
        mapFrame = tk.Canvas(self.window, highlightbackground="blue")
        console = tk.Entry(self.window, highlightbackground="blue")

        #BUTTONS
        sendCommandButton = tk.Button(self.window, text="Send command",
            command=lambda:self.tasks.put(Task.SEND, console.get()))

        modeInfo = tk.Label(infoFrame, text="Driving mode: " + self.drivingMode)
        speedInfo = tk.Label(infoFrame, text = "Speed: " + str(self.carSpeed) +
                " m/s")

        mapLabel = tk.Label(mapFrame, text="MAP")
        console.insert(0, "Enter command")
        console.bind('<Button-1>', self.clear(console))

        speedInfo.grid(row=0, column=1)
        modeInfo.grid(row=1, column=1)

        infoFrame.grid(row=0, column=0)
        mapFrame.grid(row=0, column=1)
        console.grid(row=1, column=1)
        sendCommandButton.grid(row=1, column=2)

        #MENU
        menuBar = tk.Menu(self.window)

        map_menu = tk.Menu(menuBar)
        map_menu.add_command(label="Open", command=self.open_map)
        map_menu.add_command(label="Create", command=self.create_map)
        
        system_menu = tk.Menu(menuBar)
        
        server_menu = tk.Menu(system_menu)
        server_menu.add_command(label="Enter IP-address", command=self.connect)
        system_menu.add_cascade(label="Server", menu=server_menu)
        
        system_menu.add_command(label="Quit",
            command=self.quit)
        menuBar.add_cascade(label="Map", menu=map_menu)
        menuBar.add_cascade(label="System", menu=system_menu)

        #self.window CONFIG
        self.window.title("SvartTaxi AB")
        #self.window.geometry("640x480")
        self.window.config(menu=menuBar)

    def main_loop(self):
        task_pair = self.tasks.get_completed(block=False)
        if task_pair:
            task, result = task_pair
            action = self.complete_actions.get(task)
            if action:
                action(*result)

        self.window.after(GUI.LOOP_DELAY, self.main_loop)

    def quit(self):
        self.tasks.put(Task.KILL)
        self.window.destroy()

    def open_map(self):
        print("Open")

    def create_map(self):
        print("Create")

    def clear(self, console):
        console.delete(0, tk.END)

    def connect(self):
        """
        tk.Label(self.window, text="Enter valid IP-address").pack()
        ip_popup = tk.Entry(GUI.window)
        ip_button = tk.Button(ip_popup, text="Apply IP", command=apply_ip)
        """

        self.tasks.put(Task.CONNECT, '127.0.0.1')
