import tkinter
from tkinter import *
from threading import Thread
from textwrap import fill
from course import Node, NodeType
from remote import Command, Client

def open_map():
    print("Open")
    
def create_map():
    print("Create")
    
def clear(console):
    console.delete(0, END)
 
def apply_ip():
    
    
def send_command(console, GUI):
    client = connect()
    
    if(client == None):
        
        Label(GUI.window, text="Enter valid IP-address").pack()
        ip_popup = Entry(GUI.window)
        ip_button = Button(ip_popup, text="Apply IP", command=apply_ip)
        
        
    else:
        client = connect()
        client.send_command(Command.SET_MISSION, [1.8, 'kör', 'ööö'])
    
def connect():
    CONN_PORT = 9849
    if len(sys.argv) < 2:
        sys.stderr.write('error: no IP address specified\n')
        return None
    
    client = Client(inet_addr, CONN_PORT)
    
    inet_addr = sys.argv[1]
    try:
        client.connect()
    except OSError as e:
        sys.stderr.write('failed to connect to comm -- {}\n'.format(e))
        return None
        
    return client

    
   
class GUI(Thread):

    def __init__(self):
        #VARIABLES
        self.carSpeed = 25
        self.drivingMode = "Auto"
        self.window = tkinter.Tk()

        #self.window COMPONENTS
        infoFrame = tkinter.Frame(self.window, highlightbackground="red")
        mapFrame = tkinter.Canvas(self.window, highlightbackground="blue")
        console = Entry(self.window, highlightbackground="blue")

        #BUTTONS
        sendCommandButton = tkinter.Button(self.window, text="Send command", command=send_command(console, self))

        modeInfo = Label(infoFrame, text="Driving mode: " + self.drivingMode)
        speedInfo = Label(infoFrame, text = "Speed: " + str(self.carSpeed) + " m/s")

        mapLabel = Label(mapFrame, text="MAP")
        console.insert(0, "Enter command")
        console.bind('<Button-1>', clear(console))

        speedInfo.grid(row=0, column=1)
        modeInfo.grid(row=1, column=1)

        infoFrame.grid(row=0, column=0)
        mapFrame.grid(row=0, column=1)
        console.grid(row=1, column=1)
        sendCommandButton.grid(row=1, column=2)

        #MENU
        menuBar = Menu(self.window)

        map_menu = Menu(menuBar)
        map_menu.add_command(label="Open", command=open_map)
        map_menu.add_command(label="Create", command=create_map)
        
        system_menu = Menu(menuBar)
        
        server_menu = Menu(system_menu)
        server_menu.add_command(label="Enter IP-address")
        system_menu.add_cascade(label="Server", menu=server_menu)
        
        system_menu.add_command(label="Quit", command=quit)
        menuBar.add_cascade(label="Map", menu = map_menu)
        menuBar.add_cascade(label="System", menu=system_menu)

        #self.window CONFIG
        self.window.title("SvartTaxi AB")
        #self.window.geometry("640x480")
        self.window.config(menu=menuBar)
        
    def run(self):
        self.window.mainloop()
    
def main():    
    gui = GUI()
    gui.run()
    
if __name__ == '__main__':
    main()
