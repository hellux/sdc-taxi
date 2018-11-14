'''
Created on 9 Nov 2018

@author: joe
'''

import tkinter
from tkinter import *
from textwrap import fill

def open_map():
    print("Open")
    
def create_map():
    print("Create")
    
def quit():
    exit(0)
    
#VARIABLES
carSpeed= 25
drivingMode = "Auto"

window = tkinter.Tk()
window.title("SvartTaxi AB")

infoPane = tkinter.Frame(window, bg="blue").pack(side=LEFT)
mapPane = tkinter.Frame(window).pack(side=TOP)

modeInfo = Label(infoPane, text="Driving mode: " + drivingMode)
speedInfo = Label(infoPane, text = "Speed: " + str(carSpeed) + " m/s")

mapInfo = Label(mapPane, text="MAP")
mapInfo.pack()

speedInfo.pack()
modeInfo.pack()

menuBar = Menu(window)

map_menu = Menu(menuBar)
map_menu.add_command(label="Open", command=open_map)
map_menu.add_command(label="Create", command=create_map)

system_menu = Menu(menuBar)
system_menu.add_command(label="Quit", command=quit)

menuBar.add_cascade(label="Map", menu = map_menu)
menuBar.add_cascade(label="System", menu=system_menu)

window.geometry("640x480")
window.config(menu=menuBar)
window.mainloop()