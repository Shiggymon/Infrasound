from PyQt6.QtCore import Qt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from multiprocessing import Process, Event, Queue, Pipe, connection
from multiprocessing.shared_memory import ShareableList
import numpy as np
import time
import pyqtgraph as pg
from PyQt6 import QtCore,QtWidgets
from AppMainWindow import MainWindow, MsgType, WindowMessage
from typing import NamedTuple

class SerialPortSettings(NamedTuple):
    port: str
    baudrate: int
    timeout: int = 5


def getInData(events,q:Queue,portSettings:SerialPortSettings):
    firstConversion = True
    # initialize serial port
    ser = serial.Serial()
    ser.port = portSettings.port
    ser.baudrate = portSettings.baudrate
    ser.timeout = portSettings.timeout
    ser.open()
    if ser.is_open == True:
        print("Serial port is open\n")
        print(ser, "\n")
    else:
        events["failure"].set()
    time.sleep(0.1)
    try:
        ser.reset_input_buffer()
        while not events["terminate"].is_set():
            # aquire and parse data from serial port
            recvData = []
            if ser.in_waiting:
                line = ser.readline()
                digitLine = line.strip()
                if digitLine:
                    if firstConversion:
                        firstConversion = False
                    else:
                        value = float(digitLine)
                        recvData.append(value)
            if len(recvData) > 0:
                q.put(recvData)
            time.sleep(0.01)
    except:
        events["failure"].set()
    if ser.is_open:
         ser.close()

def outData(p:connection,q:Queue):
    app = QtWidgets.QApplication([])
    main = MainWindow(p,q)
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
    dataQ = Queue()
    displayPipeC, displayPipeP = Pipe()
    serialEvents = dict()
    serialEvents["terminate"] = Event()
    serialEvents["failure"] = Event()
    inProcess = Process()
    
    outProcess = Process(None, target=outData, args=(displayPipeC,dataQ,))
    outProcess.start()
    
    while True:
        # check for events from serial
        if serialEvents["failure"].is_set():
            # serial communication failed for some reason. Terminate Thread
            serialEvents["terminate"].set()
            inProcess.join()
            serialEvents["failure"].clear()
            serialEvents["terminate"].clear()

        # check for messages from Main Window
        while displayPipeP.poll():
            msg = displayPipeP.recv()
            if msg.type == MsgType.STARTSERIAL:
            # start serial
                if not inProcess.is_alive():
                    portSettings = msg.payload
                    inProcess = Process(None, target=getInData, args=(serialEvents, dataQ, portSettings))
                    inProcess.start()
            elif msg.type == MsgType.STOPSERIAL:
                # stop serial
                serialEvents["terminate"].set()
                inProcess.join()
                serialEvents["terminate"].clear()


if __name__ == "__main__":
        main()
