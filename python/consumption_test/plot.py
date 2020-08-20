import tkinter as tk
from tkinter import ttk
from serial.tools import list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import serial
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
import csv

app = tk.Tk() 
app.minsize(400,200)
app.title("Serial ammeter port select.")
labelTop = tk.Label(app, text = "Choose ammeter port")
labelTop.grid(column=0, row=0)
combo = ttk.Combobox(app, values=list_ports.comports())
combo.grid(column=0, row=1)
combo.current(0)

def setPort():
        global port
        port = combo.get()
        app.destroy()
        
button = ttk.Button(app,text =  "Set port", command = setPort).grid(column = 0, row = 2)

app.mainloop()

ser = serial.Serial()
ser.port = port.split()[0]
ser.baudrate = 4800
ser.timeout = 10 
ser.open()
if ser.is_open==True:
	print("\nAll right, serial port now open. Configuration:\n")
	print(ser, "\n") #print serial parameters

xs = []
ys = []
fig = plt.figure()
line, = pyplot.plot(xs, ys)
with open('amps.csv', mode='w') as amp_file:
        amp_file.write("")
for i in range(0,10):
    ser.readline()

def update(frame):
    lineList = ser.readline().split(b',')
    lineList =  [float(i) for i in lineList]
    with open('amps.csv', mode='a') as amp_file:
        amps = csv.writer(amp_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        amps.writerow([lineList[0], lineList[1]])
    xs.append(lineList[0])
    ys.append(lineList[1])
    line.set_data(xs, ys)
    fig.gca().relim()
    fig.gca().autoscale_view()
    return line,
animation = FuncAnimation(fig, update,interval = 10)

pyplot.show()