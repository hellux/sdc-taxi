from __future__ import print_function
import tkinter as tk
from course import Node, NodeType, Edge
from tasks import Task
from tkinter import *
import os
import pickle
import time

class Map():
    def __init__(self, window, map_frame):
        self.window = window
        self.map_frame = map_frame
        self.map_frame.bind('<Button-1>', self.select)
        self.map_frame.bind('<Button-3>', self.node_options)
        self.nodes = []    
        self.edges = []
        self.selected_node = None
        self.selected_edge = None
        
    def node_options(self, event):
        node_options = tk.Menu(self.window, tearoff=False)
        node_create = Menu(node_options, tearoff=False)
        node_options.add_cascade(label="Create", menu=node_create)
        node_create.add_command(label="Stopline", command=lambda: self.create_node(NodeType.STOPLINE, event.x, event.y, "red"))
        node_create.add_command(label="Parking", command=lambda: self.create_node(NodeType.PARKING, event.x, event.y, "yellow"))
        node_create.add_command(label="Roundabout", command=lambda: self.create_node(NodeType.ROUNDABOUT, event.x, event.y, "blue"))
        node_options.add_command(label="Delete", command=lambda:self.delete())
        node_options.tk_popup(event.x+self.window.winfo_rootx(), event.y+self.window.winfo_rooty()) 
     
    def draw(self):
        self.map_frame.delete("all")
        
        for edge in self.edges:
            if(edge == self.selected_edge):
                self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill="green", width=2)
            else:
                self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill=edge.color, width=2)
        
        for node in self.nodes:
            if(node == self.selected_node):
               self.map_frame.create_oval(node.pos_x-5, node.pos_y-5, node.pos_x+5, node.pos_y+5, fill="green", width=2)
            else:
                self.map_frame.create_oval(node.pos_x-5, node.pos_y-5, node.pos_x+5, node.pos_y+5, fill=node.color, width=2)    
    
    def get_node(self, x, y):
        for node in self.nodes:
            if (x-5 < node.pos_x < x+5) and (y-5 < node.pos_y < y+5):
                return node
        return None
        
    def get_edge(self, x, y):
        for edge in self.edges:
            if(edge.end.pos_x-edge.start.pos_x != 0):
                k = (edge.end.pos_y-edge.start.pos_y)/(edge.end.pos_x-edge.start.pos_x)
                m = edge.start.pos_y-k*edge.start.pos_x
                if abs(y-(x*k + m)) < 5:
                    return edge
        return None
        
    def select(self, event):
        self.selected_edge = self.get_edge(event.x, event.y)

        if self.selected_node:
            self.create_edge(self.selected_node, self.get_node(event.x, event.y))
        else:    
            self.selected_node = self.get_node(event.x, event.y)
        self.draw()
        
    def delete(self):
        if(self.selected_node in self.nodes):
            self.nodes.remove(self.selected_node)
        if self.selected_edge in self.edges:
            self.edges.remove(self.selected_edge)
        self.draw()
    
    def create_edge(self, node_start, node_end):
        if(node_start and node_end):
            cost_popup = tk.Frame(self.window)
            cost_label = tk.Label(cost_popup, text="Cost: ").pack()
            cost_entry = tk.Entry(cost_popup).pack()
            #cost_popup.tk_popup(node_end.pos_x+self.window.winfo_rootx(), node_end.pos_y+self.window.winfo_rooty()) 
            edge = GraphEdge(node_start, node_end)
            self.edges.append(edge)
            self.selected_node = None
        else:
            self.selected_node = None
            
    def create_node(self, nodetype, x, y, color):
        node = GraphNode(nodetype, x, y, color)
        self.nodes.append(node)
        self.draw()
        
    def clear_map(self):
        self.edges.clear()
        self.nodes.clear()
        self.draw()
        
class GraphNode(Node):
    def __init__(self, node_type, pos_x, pos_y, color="black"):
        super().__init__(node_type)
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.color = color
        
class GraphEdge(Edge):
    def __init__(self, start, end, cost=0, color="black"):
        super().__init__(start, end, cost)
        self.color = color
        
class GUI():
    LOOP_DELAY = 50
    PREFIX_SPEED = "Speed: "
    PREFIX_MODE = "Mode: "

    def __init__(self, tasks):
        self.tasks = tasks

        self.complete_actions = {
            Task.CONNECT    : print,
            Task.SEND       : print,
            Task.GET_SENSOR : self.set_sensor,
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
        self.file = None
        
        self.car_speed.set(GUI.PREFIX_SPEED)
        self.driving_mode.set(GUI.PREFIX_MODE)

        #self.window COMPONENTS
        info_frame = tk.Frame(self.window, highlightbackground="red")
        self.map_frame = tk.Canvas(self.window, highlightbackground="black")
        console = tk.Entry(self.window, highlightbackground="blue")

        #MAP
        self.map = Map(self.window, self.map_frame)
        
        #BUTTONS
        sendCommandButton = tk.Button(self.window, text="Send command",
            command=lambda:self.tasks.put(Task.SEND, console.get()))

        car_label = tk.Label(info_frame, text="Car info")
        drive_label = tk.Label(info_frame, textvariable=self.driving_mode)
        speed_label = tk.Label(info_frame, textvariable=self.car_speed)

        car_label.grid(row=0, column=0)
        speed_label.grid(row=1, column=0)
        drive_label.grid(row=2, column=0)

        info_frame.grid(row=0, column=0)
        self.map_frame.grid(row=0, column=1)
        console.grid(row=1, column=1)
        sendCommandButton.grid(row=1, column=2)

        #MENU
        menuBar = tk.Menu(self.window)

        map_menu = tk.Menu(menuBar, tearoff=False)
        map_menu.add_command(label="Open map", command=lambda:self.get_filename("open"))
        map_menu.add_command(label="Save map", command=lambda:self.get_filename("save"))
        map_menu.add_command(label="Clear", command=self.map.clear_map)
        
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
        
    def main_loop(self):
        task_pair = self.tasks.get_completed(block=False)
        if task_pair:
            task, result = task_pair
            action = self.complete_actions.get(task)
            if action:
                action(*result)

        self.window.after(GUI.LOOP_DELAY, self.main_loop)
        self.window.after(0.01, lambda:self.tasks.put(Task.GET_SENSOR))
        
    def button_down(self, event, direction):
        self.keys[direction] = True
        self.tasks.put(Task.MOVE, self.keys.copy(), time.time())
        
    def button_up(self, event, direction):
        self.keys[direction] = False
        self.tasks.put(Task.MOVE, self.keys.copy(), time.time())
        
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
        self.unbind_keys()
        self.tasks.put(Task.SET_AUTO, True)
        self.driving_mode.set(GUI.PREFIX_MODE + "Auto")
        
    def drive_manual(self):
        self.tasks.put(Task.SET_AUTO, False)
        self.bind_keys()
        self.window.focus_set()
        self.driving_mode.set(GUI.PREFIX_MODE + "Manual")
    
    def set_sensor(self, speed_data):
        self.car_speed.set(GUI.PREFIX_SPEED + speed_data[3])
        
    def quit(self):
        self.tasks.put(Task.KILL)
        self.window.destroy()

    def get_filename(self, function):
        self.filename_frame = tk.Tk()
        filename_label = tk.Label(self.filename_frame, text="Enter filename")
        filename_entry = tk.Entry(self.filename_frame)
        if(function=="save"):
            filename_button = tk.Button(self.filename_frame, text=function, \
                command=lambda:self.save_map(filename_entry.get()))
        elif(function=="open"):
            filename_button = tk.Button(self.filename_frame, text=function, \
                command=lambda:self.open_map(filename_entry.get()))
        filename_label.grid(row=0, column=0)
        filename_entry.grid(row=1, column=0)
        filename_button.grid(row=2, column=0)
        filename_entry.focus_force()
        
    def save_map(self, filename):
        self.file = open(filename, 'wb')
        pickle.dump((self.map.nodes, self.map.edges), self.file)
        self.file.close()
        self.filename_frame.destroy()

    def open_map(self, filename):
        self.file = open(filename, 'rb')
        self.map.clear_map()
        self.map.nodes, self.map.edges = pickle.load(self.file)
        self.map.draw()
        self.filename_frame.destroy()

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
        ip_input.focus_force()
