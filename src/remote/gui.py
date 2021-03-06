from __future__ import print_function

import os
import pickle
import time
import tkinter as tk

from course import *
from tasks import Task

class Map():
    NODE_SIZE = 8
    
    def __init__(self, window, map_frame, tasks):
        self.window = window
        self.map_frame = map_frame
        self.tasks = tasks
        self.map_frame.bind('<Button-1>', self.select)
        self.map_frame.bind('<Button-3>', self.node_options)
        self.nodes = []    
        self.edges = []
        self.selected_node = None
        self.selected_edge = None
        self.select_mission = False
        self.mission_node = None
        self.path = []
        self.sub_path = []
        
    def node_options(self, event):
        node_options = tk.Menu(self.window, tearoff=False)
        node_create = tk.Menu(node_options, tearoff=False)
        node_options.add_cascade(label="Create", menu=node_create)
        node_create.add_command(label="Stopline", command=lambda: self.create_node(NodeType.STOPLINE, event.x, event.y, "red"))
        node_create.add_command(label="Parking", command=lambda: self.create_node(NodeType.PARKING, event.x, event.y, "yellow"))
        node_create.add_command(label="Inner", command=lambda: self.create_node(NodeType.INNER, event.x, event.y, "navy"))
        node_create.add_command(label="Outer", command=lambda: self.create_node(NodeType.OUTER, event.x, event.y, "RoyalBlue2"))
        node_create.add_command(label="Empty", command=lambda: self.create_node(NodeType.EMPTY, event.x, event.y, "white"))
        node_options.add_command(label="Delete", command=lambda:self.delete())
        node_options.tk_popup(int(self.map_frame.canvasx(self.map_frame.winfo_pointerx())), int(self.map_frame.canvasx(self.map_frame.winfo_pointery())))
     
    def draw(self):
        self.map_frame.delete("all")
        mission_edges = self.edges_to_highlight()
        
        for edge in self.edges:
            if(edge == self.selected_edge):
                self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill="green", width=8, arrow=tk.LAST, arrowshape=(20, 35, 7))
            else:
                self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill=edge.color, width=8, arrow=tk.LAST, arrowshape=(20, 35, 7))
        
        if mission_edges:
            print("sub_path: ", len(self.sub_path))
            for edge in mission_edges:
                self.map_frame.create_line(edge.start.pos_x, edge.start.pos_y, edge.end.pos_x, edge.end.pos_y, fill="deep pink", width=8, arrow=tk.LAST, arrowshape=(20, 35, 7))
                
        for node in self.nodes:
            if(node == self.selected_node):
               self.map_frame.create_oval(node.pos_x-Map.NODE_SIZE, node.pos_y-Map.NODE_SIZE, node.pos_x+Map.NODE_SIZE, node.pos_y+Map.NODE_SIZE, fill="green", width=2)
            else:
                self.map_frame.create_oval(node.pos_x-Map.NODE_SIZE, node.pos_y-Map.NODE_SIZE, node.pos_x+Map.NODE_SIZE, node.pos_y+Map.NODE_SIZE, fill=node.color, width=2)     
                
    def get_node(self, x, y):
        for node in self.nodes:
            if (x-Map.NODE_SIZE < node.pos_x < x+Map.NODE_SIZE) and (y-Map.NODE_SIZE < node.pos_y < y+Map.NODE_SIZE):
                return node
        return None
        
    def get_edge_pos(self, x, y):
        for edge in self.edges:
            if (min(edge.start.pos_x, edge.end.pos_x)+8 < x < max(edge.start.pos_x, edge.end.pos_x)-8):
                if (edge.end.pos_x-edge.start.pos_x != 0):
                    k = (edge.end.pos_y-edge.start.pos_y)/(edge.end.pos_x-edge.start.pos_x)
                    m = edge.start.pos_y-k*edge.start.pos_x
                    if abs(y-(x*k + m)) < Map.NODE_SIZE:
                        return edge

            if (min(edge.start.pos_y, edge.end.pos_y)+8 < y < max(edge.start.pos_y, edge.end.pos_y)-8):
                if edge.end.pos_y-edge.start.pos_x != 0:
                    k = (edge.end.pos_x-edge.start.pos_x)/(edge.end.pos_y-edge.start.pos_y)
                    m = edge.start.pos_x-k*edge.start.pos_y
                    if abs(x-(y*k + m)) < Map.NODE_SIZE:
                        return edge
        return None
        
    def edge_exists(self, start, end):
        if end not in [edge.end for edge in start.outgoing]: 
            if start not in [edge.end for edge in end.outgoing]:
                return False
        return True         
        
    def select(self, event):
        self.selected_edge = self.get_edge_pos(event.x, event.y)
        current_node = self.get_node(event.x, event.y)
        
        if self.select_mission and self.selected_node:
            self.select_mission = False
            new_path = closest_path(self.nodes, self.selected_node, current_node)
            new_mission = create_mission(clear_empty(new_path))
            #self.path = clear_empty(self.path)
            
            self.tasks.put(Task.SEND_MISSION, new_mission) 
            if self.path:
                self.path += new_path[1:]
            else:
                self.path += new_path

            path_type = []
            for node in self.path:
                path_type += [node.type]

            self.selected_node = None
            self.selected_edge = None
            print("MISSION SET")
            print("mission: ", new_mission)
            print("path: ", path_type)
            
        elif self.selected_node and current_node:
            if not self.edge_exists(self.selected_node, current_node) and self.selected_node != current_node:
                self.get_edge_cost(self.selected_node, current_node)
                
        else:    
            self.selected_node = current_node
            
        self.draw()
    
    def delete(self):
        if(self.selected_node in self.nodes):
            self.nodes.remove(self.selected_node)
        if self.selected_edge in self.edges:
            self.selected_edge.start.outgoing.remove(self.selected_edge)
            self.edges.remove(self.selected_edge)
        for edge in self.edges:
                if edge.start not in self.nodes and edge.end not in self.nodes:
                    self.edges.remove(edge)

        self.selected_node = None
        self.selected_edge = None
        self.draw()
    
    def get_edge_cost(self, node_start, node_end):
        self.cost_popup = tk.Tk()
        cost_label = tk.Label(self.cost_popup, text="Enter cost: ")
        self.cost_entry = tk.Entry(self.cost_popup)
        self.cost_popup.bind("<Return>", lambda e:self.create_edge(node_start, node_end, float(self.cost_entry.get())))
        cost_label.pack()
        self.cost_entry.pack()
        self.cost_popup.geometry("+%d+%d" % ((int(self.cost_popup.winfo_pointerx()), int((self.cost_popup.winfo_pointery())))))
        self.cost_entry.focus_force()
            
    def edges_to_highlight(self):
        highlight_edges = []
        print("subpath: ", len(self.sub_path))
        if self.sub_path:
            rest = self.sub_path
            while rest:
                node, rest = rest[0], rest[1:]
                if rest:
                    for edge in node.outgoing:
                        if edge.end == rest[0]:
                            highlight_edges.append(edge)
        return highlight_edges

    def highlight_pos(self, remaining):
        nodes = clear_empty(self.path)
        print("remaining:", remaining)
        print("nodes: ", len(nodes))
        remaining_nodes = nodes[-(remaining+1):]
        print("rem:", len(remaining_nodes))

        if len(remaining_nodes) >= 2:
            current_node = remaining_nodes[0]
            next_node = remaining_nodes[1]
            self.sub_path = []
            curr = self.path.index(current_node)
            while current_node != next_node:
                self.sub_path.append(current_node)
                current_node = self.path[curr]
                curr += 1
            self.sub_path.append(next_node)
        else:
            self.sub_path = []

        self.draw()

    def create_edge(self, node_start, node_end, cost):
        if cost < 0:
            print("INPUT POSITIVE COST")
            self.cost_popup.destroy()
            self.get_edge_cost(node_start, node_end)
            
        elif(node_start and node_end):
            edge = GraphEdge(node_start, node_end, cost)
            self.edges.append(edge)
            node_start.addEdge(edge)
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
    def __init__(self, type, pos_x, pos_y, color="black"):
        super().__init__(type)
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.color = color
        
class GraphEdge(Edge):
    def __init__(self, start, end, cost=0, color="black"):
        super().__init__(start, end, cost)
        self.color = color
        
class GUI():
    LOOP_DELAY = 50
    SENSOR_DELAY = 500
    PREFIX_MODE = "Mode: "

    def __init__(self, tasks):
        self.tasks = tasks
        self.window = tk.Tk()
        self.init_gui()
        self.window.protocol('WM_DELETE_WINDOW', self.quit)
        
        self.window.after(0, self.main_loop)
        self.window.after(0, self.get_sensor_data)
        self.window.after(0, self.update_current_node_mission)
        
        self.complete_actions = {
            Task.CONNECT    : print,
            Task.SEND       : print,
            Task.GET_SENSOR : self.display_info,
            Task.GET_MISSION: self.map.highlight_pos
        }
    def init_gui(self):
    
        #VARIABLES
        self.driving_mode = tk.StringVar()
        self.car_distance = tk.StringVar()
        self.keys = {"LEFT":False, "RIGHT":False, "FORWARD":False, "REVERSE":False}
        self.file = None
        self.cost_is_showing = False
        self.manual = True
        self.prev_cmd = []
        self.cmd_index = 0
        
        self.info_list = tk.Listbox(self.window, highlightbackground="black")
        
        self.center_x = (self.window.winfo_screenwidth() - self.window.winfo_reqwidth()) / 2
        self.center_y = (self.window.winfo_screenheight() - self.window.winfo_reqheight()) / 2
        
        self.driving_mode.set(GUI.PREFIX_MODE)

        #self.window COMPONENTS
        self.map_frame = tk.Canvas(self.window, highlightbackground="black")
        self.console = tk.Entry(self.window, text="Enter command", highlightbackground="black")

        #MAP
        self.map = Map(self.window, self.map_frame, self.tasks)
        
        #BUTTONS
        sendCommandButton = tk.Button(self.window, text="Send command", command=self.send_command)

        self.mode_label = tk.Label(self.window, textvariable=self.driving_mode)
  
        #LAYOUT
        self.mode_label.pack(side=tk.TOP)
        self.info_list.pack(side=tk.LEFT, fill="both", padx=10, pady=10)
        self.map_frame.pack(fill="both", expand=True, pady=10, padx=10, side=tk.TOP)
        sendCommandButton.pack(side=tk.BOTTOM, padx=10)
        self.console.pack(side=tk.BOTTOM, fill="both", expand=True, pady=10, padx=10)
        
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
        driving_menu.add_command(label="Set KP/KD", command=self.set_kd_kp)
        
        mission_menu = tk.Menu(menuBar, tearoff=False)
        mission_menu.add_command(label="Set mission", command=self.set_mission)
        mission_menu.add_command(label="Clear mission", command=self.clear_mission)
        
        menuBar.add_cascade(label="Map", menu=map_menu)
        menuBar.add_cascade(label="Mission", menu=mission_menu)
        menuBar.add_cascade(label="Driving", menu=driving_menu)
        menuBar.add_cascade(label="System", menu=system_menu)

        self.window.title("SvartTaxi AB")
        self.window.config(menu=menuBar)
        self.window.geometry("+%d+%d" % (self.center_x, self.center_y))
        
        #KEYBOARD BINDINGS
        self.window.bind('<KeyPress-Control_L>', self.show_edge_cost)
        self.console.bind('<Return>', lambda e:self.send_command())
        self.console.bind('<Up>', self.send_prev_cmd)
        self.console.bind('<Down>', self.send_next_cmd)
        
    def main_loop(self):
        task_pair = self.tasks.get_completed(block=False)
        if task_pair:
            task, result = task_pair
            action = self.complete_actions.get(task)
            if action:
                action(*result)

        self.window.after(GUI.LOOP_DELAY, self.main_loop)
    
    def set_kd_kp(self):
        self.kp_kd_frame = tk.Tk()
        
        kp_vel_label = tk.Label(self.kp_kd_frame, text="KP VEL")
        kd_vel_label = tk.Label(self.kp_kd_frame, text="KD VEL")
        kd_rot_label = tk.Label(self.kp_kd_frame, text="KD ROT")
        kp_rot_label = tk.Label(self.kp_kd_frame, text="KP ROT")
        
        kp_vel_entry = tk.Entry(self.kp_kd_frame)
        kd_vel_entry = tk.Entry(self.kp_kd_frame)
        kp_rot_entry = tk.Entry(self.kp_kd_frame)
        kd_rot_entry = tk.Entry(self.kp_kd_frame)
        
        kp_vel_label.pack()
        kp_vel_entry.pack()
        kd_vel_label.pack()
        kd_vel_entry.pack()
        kp_rot_label.pack()
        kp_rot_entry.pack()
        kd_rot_label.pack()
        kd_rot_entry.pack()
        self.kp_kd_frame.geometry("+%d+%d" % ((int(self.kp_kd_frame.winfo_pointerx()), int((self.kp_kd_frame.winfo_pointery())))))
        self.kp_kd_frame.bind("<Return>", lambda e:self.send_kd_kp(kp_vel_entry.get(), kd_vel_entry.get(), kd_rot_entry.get(), kp_rot_entry.get()))
        kp_vel_entry.focus_force()
      
    def send_kd_kp(self, kp_vel, kd_vel, kd_rot, kp_rot):
        self.tasks.put(Task.SET_VEL, kp_vel, kd_vel)
        self.tasks.put(Task.SET_ROT, kp_rot, kd_rot)
        self.kp_kd_frame.destroy()
        
    def drive_auto(self):
        self.unbind_keys()
        self.tasks.put(Task.SET_AUTO, True)
        self.driving_mode.set(GUI.PREFIX_MODE + "Auto")
        
    def drive_manual(self):
        self.tasks.put(Task.SET_AUTO, False)
        self.bind_keys()
        self.window.focus_set()
        self.driving_mode.set(GUI.PREFIX_MODE + "Manual")
        
    def send_command(self):
        self.cmd_index = 0
        self.tasks.put(Task.SEND, self.console.get())
        self.prev_cmd.insert(0, self.console.get())
        self.prev_cmd = self.prev_cmd[0:9]
        self.console.delete(0, 'end')
 
    def send_prev_cmd(self, event):
        if self.cmd_index >= len(self.prev_cmd)-1:
            self.cmd_index = 0
        else:            
            self.cmd_index += 1

        self.console.delete(0, 'end')
        self.console.insert(0, self.prev_cmd[self.cmd_index])
    
    def send_next_cmd(self, event):
        if self.cmd_index <= 0:
            self.cmd_index = len(self.prev_cmd)-1
        else:
            self.cmd_index -= 1

        self.console.delete(0, 'end')
        self.console.insert(0, self.prev_cmd[self.cmd_index])
            
    def get_sensor_data(self):
        self.tasks.put(Task.GET_SENSOR)
        self.window.after(GUI.SENSOR_DELAY, self.get_sensor_data)
        
    def update_current_node_mission(self):
        self.tasks.put(Task.GET_MISSION)
        self.window.after(GUI.SENSOR_DELAY, self.update_current_node_mission)
        
    def set_mission(self):
        print("SELECT START AND DESTINATION")
        self.map.select_mission = True
    
    def clear_mission(self): 
        self.map.path = []
        self.map.mission = []
        self.map.sub_path = []
        self.tasks.put(Task.CLEAR_MISSION)
        self.map.selected_node = None
        self.map.selected_edge = None
        self.map.draw()

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

    def display_info(self, sensor_data):
        self.map.total_distance = float(sensor_data[3])
        self.info_list.delete(0, 'end')
        info_labels = ["Front", "Right", "Speed", "Distance", "Error",
                       "set speed", "set rotation"]
        for info in info_labels:
            self.info_list.insert(info_labels.index(info), info + ": " + sensor_data[info_labels.index(info)])
        
    def quit(self):
        self.tasks.put(Task.KILL)
        self.window.destroy()
        
    def show_edge_cost(self, event):
        if not self.cost_is_showing:
            for edge in self.map.edges:
                self.map_frame.lift(self.map_frame.create_text((edge.start.pos_x+edge.end.pos_x)/2, (edge.start.pos_y+edge.end.pos_y)/2+10, text=edge.cost))
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
            self.filename_frame.bind("<Return>", lambda e:self.save_map(filename_entry.get()))
        elif(function=="open"):
           self.filename_frame.bind("<Return>", lambda e:self.open_map(filename_entry.get()))
        filename_label.pack()
        filename_entry.pack()
        self.filename_frame.geometry("+%d+%d" % ((int(self.filename_frame.winfo_pointerx()), int((self.filename_frame.winfo_pointery())))))
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
        
    def apply_ip(self, address):
        self.tasks.put(Task.CONNECT, address)
        self.ip_popup.destroy()
                       
    def connect(self):
        self.ip_popup = tk.Tk()
        self.ip_popup.title("Connect to server")
        ip_label = tk.Label(self.ip_popup, text="Enter IP-address")
        ip_input = tk.Entry(self.ip_popup)
        self.ip_popup.bind("<Return>", lambda e: self.apply_ip(ip_input.get()))
        ip_label.pack()
        ip_input.pack()
        self.ip_popup.geometry("+%d+%d" % ((int(self.ip_popup.winfo_pointerx()), int((self.ip_popup.winfo_pointery())))))
        ip_input.focus_force()
