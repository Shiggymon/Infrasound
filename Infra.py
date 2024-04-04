from PyQt6.QtCore import Qt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from multiprocessing import Process, Event, Queue
from multiprocessing.shared_memory import ShareableList
import numpy as np
import time
import pyqtgraph as pg
from PyQt6 import QtCore,QtWidgets
from AppMainWindow import MainWindow

def getInData(q:Queue):
    # initialize serial port
    ser = serial.Serial()
    ser.port = "/dev/tty.usbserial-A5XK3RJT"
    ser.baudrate = 38400
    ser.timeout = 10
    ser.open()
    if ser.is_open == True:
        print("Serial port is open\n")
        print(ser, "\n")
    time.sleep(1)
    while True:
        #ser.reset_input_buffer()
        # aquire and parse data from serial port
        recvData = []
        if ser.in_waiting:
            line = ser.readline()
            value = float(line)
            recvData.append(value)
            # add x and y to the list
            # if len(xs) > 0:
            #     xs.append(xs[-1]+1)
            # else:
            #     xs.append(0)
            # ys.append(value)
        if len(recvData) > 0:
            q.put(recvData)
        # limit to 50*10 items
        #xs = xs[-500:]
        #ys = ys[-500:]
        time.sleep(0.01)
#        break
        
def outData(q:Queue):
#    fig = plt.figure()
#    ax = fig.add_subplot(1,1,1)
#    line1, = ax.plot(xs,ys, label="Pa")
#    fig.canvas.draw()
#    ani = animation.FuncAnimation(fig, displayGraph, fargs=(q, xs, ys), interval=100)
#    plt.show()
    app = QtWidgets.QApplication([])
    main = MainWindow(q)
    main.show()
    app.exec()

def q2Vars(q, xs, ys):
    while True:
        try:
            item = q.get(block=False)
            for i in item:
                xs.append(xs[-1]+1)
                ys.append(i)
        except:
            break
    while(len(xs)>500):
        xs.pop(0)
        ys.pop(0)

def displayGraph(i, q, xs, ys):
    q2Vars(q, xs, ys)
    plt.gca().clear()
    plt.gca().plot(xs, ys)

def main():    
    xs = [0]
    ys = [0]
    q = Queue()
    # create figure to plot into

    #ax.clear()
    #ax.plot(xs,ys,label="Pa")
    time.sleep(5)

    inProcess = Process(None, target=getInData, args=(q,))
    inProcess.start()
    
    outProcess = Process(None, target=outData, args=(q,))
    outProcess.start()
        
    inProcess.join()

if __name__ == "__main__":
        main()