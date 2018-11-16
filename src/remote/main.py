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
    
def clear(event):
    console.delete(0, END)
    
def send_command():
    console.delete(0, END)
    print(str(console.get()))
    #TODO: Skicka kommando till RPi
    
#VARIABLES
carSpeed = 25
drivingMode = "Auto"
window = tkinter.Tk()

#WINDOW COMPONENTS
infoFrame = tkinter.Frame(window, highlightbackground="red")
mapFrame = tkinter.Canvas(window, highlightbackground="blue")
console = Entry(window, highlightbackground="blue")

#BUTTONS
sendCommandButton = tkinter.Button(window, text="Send command", command=send_command)

modeInfo = Label(infoFrame, text="Driving mode: " + drivingMode)
speedInfo = Label(infoFrame, text = "Speed: " + str(carSpeed) + " m/s")

mapLabel = Label(mapFrame, text="MAP")
console.insert(0, "Enter command")
console.bind('<Button-1>', clear)

speedInfo.grid(row=0, column=1)
modeInfo.grid(row=1, column=1)

infoFrame.grid(row=0, column=0)
mapFrame.grid(row=0, column=1)
console.grid(row=1, column=1)
sendCommandButton.grid(row=1, column=2)

#MENU
menuBar = Menu(window)

map_menu = Menu(menuBar)
map_menu.add_command(label="Open", command=open_map)
map_menu.add_command(label="Create", command=create_map)

system_menu = Menu(menuBar)
system_menu.add_command(label="Quit", command=quit)

menuBar.add_cascade(label="Map", menu = map_menu)
menuBar.add_cascade(label="System", menu=system_menu)

#WINDOW CONFIG
window.title("SvartTaxi AB")
#window.geometry("640x480")
window.config(menu=menuBar)
window.mainloop()