from __future__ import print_function
import tkinter as tk
from course import Node, NodeType
from tasks import Task
from tkinter import *
import os

class Map():
    
    def __init__(self, window, map_frame):
        self.window = window
        self.map_frame = map_frame
        self.map_frame.bind('<Button-1>', self.draw_node)
        self.course = []
        
    def draw_node(self, event):
        self.node_popup(event)
        self.map_frame.create_oval(event.x-5, event.y-5, event.x+5, event.y+5, fill="black", width=2)
    
    def node_popup(self, event):
        node_menu = tk.Menu(self.window, tearoff=False)
        node_menu.add_command(label="STOPLINE", command=lambda: self.create_node(NodeType.STOPLINE, event.x, event.y))
        node_menu.add_command(label="PARKING", command=lambda: self.create_node(NodeType.PARKING, event.x, event.y))
        node_menu.add_command(label="ROUNDABOUT", command=lambda: self.create_node(NodeType.ROUNDABOUT, event.x, event.y))
        node_menu.tk_popup(event.x+self.window.winfo_x(), event.y+self.window.winfo_y())
        
    def create_node(self, nodetype, x, y):
        node = GraphNode(nodetype, x, y)
        self.course.append(node)
        for node in self.course: print(node.type)
        
class GraphNode(Node):
    def __init__(self, node_type, pos_x, pos_y):
        super().__init__(node_type)
        self.pos_x = pos_x
        self.pos_y = pos_y

class GUI():
    LOOP_DELAY = 50
    PREFIX_SPEED = "Speed: "
    PREFIX_MODE = "Driving mode: "

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
        self.car_speed = tk.StringVar()
        self.driving_mode = tk.StringVar()
        self.keys = {"LEFT":False, "RIGHT":False, "FORWARD":False, "REVERSE":False}
        
        self.car_speed.set(GUI.PREFIX_SPEED)
        self.driving_mode.set(GUI.PREFIX_MODE)

        #self.window COMPONENTS
        info_frame = tk.Frame(self.window, highlightbackground="red")
        self.map_frame = tk.Canvas(self.window, highlightbackground="black")
        console = tk.Entry(self.window, highlightbackground="blue")

        #BUTTONS
        sendCommandButton = tk.Button(self.window, text="Send command",
            command=lambda:self.tasks.put(Task.SEND, console.get()))

        drive_label = tk.Label(info_frame, textvariable=self.driving_mode)
        speed_label = tk.Label(info_frame, textvariable=self.car_speed)

        speed_label.grid(row=0, column=1)
        drive_label.grid(row=1, column=1)

        info_frame.grid(row=0, column=0)
        self.map_frame.grid(row=0, column=1)
        console.grid(row=1, column=1)
        sendCommandButton.grid(row=1, column=2)

        #MENU
        menuBar = tk.Menu(self.window)

        map_menu = tk.Menu(menuBar, tearoff=False)
        map_menu.add_command(label="Open", command=self.open_map)
        map_menu.add_command(label="Create", command=self.create_map)
        
        system_menu = tk.Menu(menuBar, tearoff=False)
        
        server_menu = tk.Menu(system_menu, tearoff=False)
        server_menu.add_command(label="Connect", command=self.connect)
        
        system_menu.add_cascade(label="Server", menu=server_menu)
        system_menu.add_command(label="Quit", command=self.quit)

        driving_menu = tk.Menu(menuBar, tearoff=False)
        driving_menu.add_command(label="Auto", command=self.drive_auto)
        driving_menu.add_command(label="Manual", command=self.drive_manual)
        
        menuBar.add_cascade(label="Map", menu=map_menu)
        menuBar.add_cascade(label="Driving", menu=driving_menu)
        menuBar.add_cascade(label="System", menu=system_menu)

        self.window.title("SvartTaxi AB")
        self.window.config(menu=menuBar)
        
        #MAP
        map = Map(self.window, self.map_frame)
        
    def main_loop(self):
        task_pair = self.tasks.get_completed(block=False)
        if task_pair:
            task, result = task_pair
            action = self.complete_actions.get(task)
            if action:
                action(*result)

        self.window.after(GUI.LOOP_DELAY, self.main_loop)

    def button_down(self, event, direction):
        self.keys[direction] = True
        self.tasks.put(Task.MOVE, self.keys.copy())
        
    def button_up(self, event, direction):
        self.keys[direction] = False
        self.tasks.put(Task.MOVE, self.keys.copy())
        
    def bind_keys(self):
        self.window.bind("<Left>", lambda e:self.button_down(e, "LEFT"))
        self.window.bind("<Right>", lambda e:self.button_down(e, "RIGHT"))
        self.window.bind("<Up>", lambda e:self.button_down(e, "FORWARD"))
        self.window.bind("<Down>", lambda e:self.button_down(e, "REVERSE"))
        
        self.window.bind("<KeyRelease-Left>", lambda e: self.button_up(e, "LEFT"))
        self.window.bind("<KeyRelease-Right>", lambda e: self.button_up(e, "RIGHT"))
        self.window.bind("<KeyRelease-Up>", lambda e: self.button_up(e, "FORWARD"))
        self.window.bind("<KeyRelease-Down>", lambda e: self.button_up(e, "REVERSE"))
        
    def unbind_keys(self):
        self.window.unbind("<Left>")
        self.window.unbind("<Right>")
        self.window.unbind("<Up>")
        self.window.unbind("<Down>")
        
    def drive_auto(self):
        os.system('xset r on')
        self.unbind_keys()
        self.tasks.put(Task.SET_AUTO, True)
        self.driving_mode.set(GUI.PREFIX_MODE + "Auto")
        
    def drive_manual(self):
        os.system('xset r off')
        self.tasks.put(Task.SET_AUTO, False)
        self.bind_keys()
        self.window.focus_set()
        self.driving_mode.set(GUI.PREFIX_MODE + "Manual")
        
    def quit(self):
        os.system('xset r on')
        self.tasks.put(Task.KILL)
        self.window.destroy()

    def open_map(self):
        print("Open")

    def create_map(self):
        print("Create")
  
    def clear(self, console):
        console.delete(0, tk.END)

    def connect(self):
        ip_popup = tk.Tk()
        ip_popup.title("Connect to server")
        ip_input = tk.Entry(ip_popup)
        ip_button = tk.Button(ip_popup, text="Connect to server", \
            command=lambda:self.tasks.put(Task.CONNECT, ip_input.get()))
        ip_input.grid(row=0, column=0)
        ip_button.grid(row=0, column=1)
