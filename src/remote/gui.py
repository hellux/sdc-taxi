from __future__ import print_function

import os
import pickle
import time
import tkinter as tk

from course import Edge, Node, NodeType
from tasks import Task


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
        self.select_mission = False
        
    def node_options(self, event):
        node_options = tk.Menu(self.window, tearoff=False)
        node_create = tk.Menu(node_options, tearoff=False)
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

        if self.select_mission and selected_node:
            self.select_mission = False
            path = Course.closest_path(self.course, self.selected_node, self.get_node(event.x, event.y))
            Course.create_mission(path)
            
        elif self.selected_node:
            self.get_edge_cost(self.selected_node, self.get_node(event.x, event.y))
            
        else:    
            self.selected_node = self.get_node(event.x, event.y)
        self.draw()
    
    def delete(self):
        if(self.selected_node in self.nodes):
            self.nodes.remove(self.selected_node)
        if self.selected_edge in self.edges:
            self.edges.remove(self.selected_edge)
        self.draw()
    
    def get_edge_cost(self, node_start, node_end):
        self.cost_popup = tk.Tk()
        cost_label = tk.Label(self.cost_popup, text="Enter cost: ")
        cost_entry = tk.Entry(self.cost_popup)
        cost_button = tk.Button(self.cost_popup, text="OK", command=lambda:self.create_edge(node_start, node_end, cost_entry.get()))
        cost_label.grid(row=0, column=0)
        cost_entry.grid(row=1, column=0)
        cost_button.grid(row=2, column=0)
        cost_entry.focus_force()

    def create_edge(self, node_start, node_end, cost):
        if(node_start and node_end):
            edge = GraphEdge(node_start, node_end, cost)
            self.edges.append(edge)
            self.selected_node = None
            self.draw()
        else:
            self.selected_node = None
        self.cost_popup.destroy()
            
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
            Task.GET_SENSOR : self.set_car_info,
        }

        self.window = tk.Tk()
        self.init_gui()
        self.window.protocol('WM_DELETE_WINDOW', self.quit)
        
        self.window.after(0, self.main_loop)
        self.window.after(0, self.get_sensor_data)

    def init_gui(self):
    
        #VARIABLES
        self.car_speed = tk.StringVar()
        self.driving_mode = tk.StringVar()
        self.car_distance = tk.StringVar()
        self.keys = {"LEFT":False, "RIGHT":False, "FORWARD":False, "REVERSE":False}
        self.file = None
        self.cost_is_showing = False
        
        self.center_x = (self.window.winfo_screenwidth() - self.window.winfo_reqwidth()) / 2
        self.center_y = (self.window.winfo_screenheight() - self.window.winfo_reqheight()) / 2
        
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

        self.mode_label = tk.Label(info_frame, textvariable=self.driving_mode)
        self.speed_label = tk.Label(info_frame, textvariable=self.car_speed)
        self.distance_label = tk.Label(info_frame, textvariable=self.car_distance)

        self.speed_label.pack(fill="both")
        self.mode_label.pack(fill="both")
        self.distance_label.pack(fill="both")
        
        info_frame.pack(fill="both")
        self.map_frame.pack(fill="both", expand=True)
        console.pack(fill="both")
        sendCommandButton.pack(fill="both")
        
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
        
        mission_menu = tk.Menu(menuBar, tearoff=False)
        mission_menu.add_command(label="Set mission", command=self.set_mission)
        
        menuBar.add_cascade(label="Map", menu=map_menu)
        menuBar.add_cascade(label="Mission", menu=mission_menu)
        menuBar.add_cascade(label="Driving", menu=driving_menu)
        menuBar.add_cascade(label="System", menu=system_menu)

        self.window.title("SvartTaxi AB")
        self.window.config(menu=menuBar)
        self.window.geometry("+%d+%d" % (self.center_x, self.center_y))
        
        #KEYBOARD BINDINGS
        self.window.bind('<KeyPress-Control_L>', self.show_edge_cost)
        
    def main_loop(self):
        task_pair = self.tasks.get_completed(block=False)
        if task_pair:
            task, result = task_pair
            action = self.complete_actions.get(task)
            if action:
                action(*result)

        self.window.after(GUI.LOOP_DELAY, self.main_loop)
        

    def get_sensor_data(self):
        self.tasks.put(Task.GET_SENSOR)
        self.window.after(1, self.get_sensor_data)
        
    def set_mission(self):
        print("SELECT START AND DESTINATION")
        self.map.select_mission = True
        
    def button_down(self, direction):
        self.keys[direction] = True
        self.tasks.put(Task.MOVE, self.keys.copy(), time.time())
        
    def button_up(self, direction):
        self.keys[direction] = False
        self.tasks.put(Task.MOVE, self.keys.copy(), time.time())
        
    def bind_keys(self):
        self.window.bind("<Left>", lambda e:self.button_down("LEFT"))
        self.window.bind("<Right>", lambda e:self.button_down("RIGHT"))
        self.window.bind("<Up>", lambda e:self.button_down("FORWARD"))
        self.window.bind("<Down>", lambda e:self.button_down("REVERSE"))
        
        self.window.bind("<KeyRelease-Left>", lambda e: self.button_up("LEFT"))
        self.window.bind("<KeyRelease-Right>", lambda e: self.button_up("RIGHT"))
        self.window.bind("<KeyRelease-Up>", lambda e: self.button_up("FORWARD"))
        self.window.bind("<KeyRelease-Down>", lambda e: self.button_up("REVERSE"))
        
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
    
    def set_car_info(self, sensor_data):
        self.car_distance.set("Distance: " + sensor_data[2])
        self.car_speed.set(GUI.PREFIX_SPEED + sensor_data[3])
        
    def quit(self):
        self.tasks.put(Task.KILL)
        self.window.destroy()
        
    def show_edge_cost(self, event):
        if not self.cost_is_showing:
            for edge in self.map.edges:
                self.map_frame.create_text((edge.start.pos_x+edge.end.pos_x)/2, (edge.start.pos_y+edge.end.pos_y)/2, text=edge.cost)
            self.cost_is_showing = True
        elif self.cost_is_showing:
            self.cost_is_showing = False
            self.map.clear_map
            self.map.draw()
            
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
        self.filename_frame.geometry("+%d+%d" % (self.center_x, self.center_y))
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
