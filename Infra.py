from PyQt6.QtCore import Qt
import serial
from multiprocessing import Process, Event, Queue, Pipe, connection, freeze_support
from multiprocessing.shared_memory import ShareableList
import numpy as np
import time
import pyqtgraph as pg
from PyQt6 import QtCore,QtWidgets
from AppMainWindow import MainWindow, MsgType, WindowMessage
from typing import NamedTuple
from random import normalvariate

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
    try:
        ser.open()
    except:
        events["failure"].set()
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
         
def getVirtualData(events,q:Queue):
    while not events["terminate"].is_set():
        # create data for serial input
        recvData = normalvariate(mu=0, sigma=3)
        scaledData = [round(recvData, 4)]
        q.put(scaledData)
        time.sleep(0.02)
        
        
        
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

def main():    
    dataQ = Queue()
    displayPipeC, displayPipeP = Pipe()
    serialEvents = dict()
    serialEvents["terminate"] = Event()
    serialEvents["failure"] = Event()
    inProcess = Process()
    
    outProcess = Process(None, target=outData, args=(displayPipeC,dataQ,))
    outProcess.start()
    mainActive = True
    while mainActive:
        # check for events from serial
        if serialEvents["failure"].is_set():
            # serial communication failed for some reason. Terminate Thread
            serialEvents["terminate"].set()
            inProcess.join()
            serialEvents["failure"].clear()
            serialEvents["terminate"].clear()
            displayPipeP.send(WindowMessage(MsgType.SERIALSTOPPED))

        # check for messages from Main Window
        while displayPipeP.poll():
            msg = displayPipeP.recv()
            if msg.type == MsgType.STARTSERIAL:
            # start serial
                if not inProcess.is_alive():
                    portSettings = msg.payload
                    if portSettings.port == "virtual":
                        inProcess = Process(None, target=getVirtualData, args=(serialEvents, dataQ))
                    else:
                        inProcess = Process(None, target=getInData, args=(serialEvents, dataQ, portSettings))
                    inProcess.start()
                    displayPipeP.send(WindowMessage(MsgType.SERIALSTARTED))
            elif msg.type == MsgType.STOPSERIAL:
                # stop serial
                serialEvents["terminate"].set()
                if inProcess.is_alive():
                    inProcess.join()
                serialEvents["terminate"].clear()
                displayPipeP.send(WindowMessage(MsgType.SERIALSTOPPED))
            elif msg.type == MsgType.STOPWINDOW:
                outProcess.join()
                if inProcess.is_alive():
                    inProcess.join()
                mainActive = False
                


if __name__ == "__main__":
    freeze_support()
    print("starting...")
    
    main()
