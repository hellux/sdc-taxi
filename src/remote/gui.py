from __future__ import print_function
import tkinter as tk
from course import Node, NodeType, Edge
from tasks import Task
from tkinter import *
import os

class Map():
    
    def __init__(self, window, map_frame):
        self.window = window
        self.map_frame = map_frame
        self.map_frame.bind('<Button-1>', self.select)
        self.map_frame.bind('<Button-3>', self.node_options)
        self.nodes = []    
        self.edges = []
        self.selected_node = None
        
    def node_options(self, event):
        node_options = tk.Menu(self.window, tearoff=False)
        node_create = Menu(node_options, tearoff=False)
        node_options.add_cascade(label="Create", menu=node_create)
        node_create.add_command(label="Stopline", command=lambda: self.create_node(NodeType.STOPLINE, event.x, event.y))
        node_create.add_command(label="Parking", command=lambda: self.create_node(NodeType.PARKING, event.x, event.y))
        node_create.add_command(label="Roundabout", command=lambda: self.create_node(NodeType.ROUNDABOUT, event.x, event.y))
        node_options.add_command(label="Delete", command=lambda:self.delete_node())
        node_options.tk_popup(event.x+self.window.winfo_rootx(), event.y+self.window.winfo_rooty()) 
     
    def draw(self):
        self.map_frame.delete("all")
        for node in self.nodes:
            if(node == self.selected_node):
               self.map_frame.create_oval(node.pos_x-5, node.pos_y-5, node.pos_x+5, node.pos_y+5, fill="red", width=2)
            else:
                self.map_frame.create_oval(node.pos_x-5, node.pos_y-5, node.pos_x+5, node.pos_y+5, fill="black", width=2)
                
        for edge in self.edges:
            self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill="black", width=2)
            
    def get_node(self, x, y):
        for node in self.nodes:
            if (x-50 < node.pos_x < x+50) and (y-50 < node.pos_y < y+50):
                return node
        return None
        
    def select(self, event):
        if self.selected_node:
            self.create_edge(self.selected_node, self.get_node(event.x, event.y))
        else:    
            self.selected_node = self.get_node(event.x, event.y)
        self.draw()
        
    def delete_node(self):
        if(self.selected_node in self.nodes):
            self.nodes.remove(self.selected_node)
            self.draw()
    
    def create_edge(self, node_start, node_end):
        if(node_start and node_end):
            edge = Edge(node_start, node_end)
            self.edges.append(edge)
            self.selected_node = None
            
    def create_node(self, nodetype, x, y):
        node = GraphNode(nodetype, x, y)
        self.nodes.append(node)
        self.draw()
        
class GraphNode(Node):
    def __init__(self, node_type, pos_x, pos_y, color="black"):
        super().__init__(node_type)
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.color = color

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
