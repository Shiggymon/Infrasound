from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QFrame, QGroupBox
from PyQt6.QtCore import QTimer
from multiprocessing import Queue, connection
import pyqtgraph as pg
from scipy.fft import fft,fftfreq
import numpy as np
import serial.tools.list_ports
from enum import Enum, auto
from typing import NamedTuple

class MsgType(Enum):
    STARTSERIAL = auto()
    STOPSERIAL = auto()
    SERIALSTARTED = auto()
    SERIALSTOPPED = auto()

class WindowMessage(NamedTuple):
    type: MsgType
    payload: object = None

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, p:connection, q:Queue) -> None:
        super().__init__()        
        self.xs = [0]
        self.ys = [0]
        self.useSPL = True # Output fft data in dBSPL instead of Pa
        self.captureActive = False
        self.capturePaused = False
        self.serialPorts = list()
        self.setWindowTitle("Infraschall Analyse")
        mainWidget = QWidget()
        vLayout = QVBoxLayout()
        hTopLayout = QHBoxLayout()
        settingsLayout = QGridLayout()
        settingsSeparator1 = QFrame()
        settingsSeparator2 = QFrame()
        plotGraph = pg.PlotWidget()
        fftGraph = pg.PlotWidget()
        buttonReloadSerial = QPushButton("Reload")
        buttonReloadSerial.clicked.connect(self.reloadSerial)
        self.comboSelectSerial = QComboBox()
        self.comboSelectSerial.setEditable(True)
        self.buttonStartCapture = QPushButton("Start")
        self.buttonStartCapture.clicked.connect(self.startCapture)
        self.buttonStopCapture = QPushButton("Stop")
        self.buttonStopCapture.clicked.connect(self.stopCapture)
        self.buttonStopCapture.setEnabled(False)
        self.buttonPauseCapture = QPushButton("Pause")
        self.buttonPauseCapture.clicked.connect(self.pauseCapture)
        self.buttonPauseCapture.setCheckable(True)
        self.buttonPauseCapture.setEnabled(False)


        # Set Top Layout with 2 Graphs
        vLayout.addLayout(hTopLayout)
        hTopLayout.addWidget(plotGraph)
        hTopLayout.addWidget(fftGraph)
        # Set Bot Layyout with a settings Grid
        vLayout.addLayout(settingsLayout)
        settingsLayout.setHorizontalSpacing(20)

        settingsLayout.addWidget(settingsSeparator1,1, 2, 6, 2)
        settingsLayout.addWidget(settingsSeparator2,1, 4, 6, 4)
        settingsSeparator1.setFrameShape(QFrame.Shape.VLine)
        settingsSeparator2.setFrameShape(QFrame.Shape.VLine)
        settingsLayout.setColumnStretch(1,1)
        settingsLayout.setColumnStretch(3,1)
        settingsLayout.setColumnStretch(5,1)
        # Settings Column 1: Serial Connection and data Recording
        settingsLayout.addWidget(QLabel("Data Capturing"),0,1)
        settingsLayout.addWidget(self.comboSelectSerial,1,1)
        settingsLayout.addWidget(buttonReloadSerial,2,1)
        settingsLayout.addWidget(QLabel(""),3,1)
        settingsLayout.addWidget(self.buttonStartCapture,4,1)
        settingsLayout.addWidget(self.buttonPauseCapture,5,1)
        settingsLayout.addWidget(self.buttonStopCapture,6,1)
        # Settings Column 3: Display Settings
        settingsLayout.addWidget(QLabel("Data Display"),0,3)
        settingsLayout.addWidget(QDoubleSpinBox(),1,3)
        settingsLayout.addWidget(QDoubleSpinBox(),2,3)
        settingsLayout.addWidget(QGroupBox(),3,3)
        

        # Settings Column 5: Saving Data
        settingsLayout.addWidget(QLabel("Data Saving"),0,5)
        settingsLayout.addWidget(QPushButton("Save (png)"),3,5)
        settingsLayout.addWidget(QPushButton("Save (csv)"),4,5)
        

        # Add Layout to Main Window
        mainWidget.setLayout(vLayout)
        self.setCentralWidget(mainWidget)
        self.lineT = plotGraph.plot(self.xs,self.ys)
        self.lineF = fftGraph.plot(self.xs,self.ys)
        self.q = q
        self.displayPipe = p

        # Setup Timers for periodic UI and Data Update
        self.updDataTimer = QTimer()
        self.updDataTimer.setInterval(50)
        self.updDataTimer.timeout.connect(self.updateData)
        self.updDataTimer.start()
        self.updPlotTTimer = QTimer()
        self.updPlotTTimer.setInterval(100)
        self.updPlotTTimer.timeout.connect(self.updateTPlot)
        self.updPlotTTimer.start()
        self.updPlotFTimer = QTimer()
        self.updPlotFTimer.setInterval(200)
        self.updPlotFTimer.timeout.connect(self.updateFPlot)
        self.updPlotFTimer.start()
        self.updCommsTimer = QTimer()
        self.updCommsTimer.setInterval(200)
        self.updCommsTimer.timeout.connect(self.updateComms)
        self.updCommsTimer.start()
        
    def q2Vars(self, q, xs, ys):
        while True:
            try:
                item = q.get(block=False)
                for i in item:
                    if not self.capturePaused:
                        xs.append(xs[-1]+1)
                        ys.append(i)
            except:
                break
        while(len(xs)>500):
            xs.pop(0)
            ys.pop(0)

    def updateData(self):
         self.q2Vars(self.q, self.xs, self.ys)

    def updateTPlot(self):
        xs = np.array(self.xs)
        ys = np.array(self.ys)
        self.lineT.setData(xs, ys)

    def updateFPlot(self):
        T = 1/50 # 50 Hz
        N = len(self.ys)
        ys = np.array(self.ys)
        yf = fft(ys)
        xf = fftfreq(N, T)[:N//2]
        if self.useSPL:
            self.lineF.setData(xf, self.lin2dbSPL(2.0 * abs(yf[0:N//2])))
        else:
            self.lineF.setData(xf, 2.0 * abs(yf[0:N//2]))
    
    def updateComms(self):
        while self.displayPipe.poll():
            msg = self.displayPipe.recv()
            try:
                if msg.type == MsgType.SERIALSTARTED:
                    self.captureActive = True
                    self.buttonStopCapture.setEnabled(True)
                    self.pauseCapture(pause=False)
                elif msg.type == MsgType.SERIALSTOPPED:
                    self.captureActive = False
                    self.buttonStartCapture.setEnabled(True)
                    self.buttonStopCapture.setEnabled(False)
                    self.buttonPauseCapture.setEnabled(False)
            except:
                raise

    def lin2dbSPL(self,x):
        return 20*np.log10(x/(2*10**-5))
    
    def reloadSerial(self):
        currentIndex = self.comboSelectSerial.currentIndex()
        currentText = self.comboSelectSerial.currentText()
        self.serialPorts = serial.tools.list_ports.comports()
        self.serialPorts.sort(key=lambda x : x.device)
        # clear list
        count = self.comboSelectSerial.count()
        for i in range(0,count):
            self.comboSelectSerial.removeItem(0)
        # repopulate list
        for p in self.serialPorts:
            self.comboSelectSerial.addItem(p.device)
        # select previously selected port. If none was selected, select first. 
        if currentIndex == -1 and currentText == "":
            self.comboSelectSerial.setCurrentIndex(0)
        else:
            self.comboSelectSerial.setCurrentText(currentText)
    
    def startCapture(self):
        from Infra import SerialPortSettings
        msg = WindowMessage(MsgType.STARTSERIAL, SerialPortSettings(port=self.comboSelectSerial.currentText(), baudrate=38400))
        self.displayPipe.send(msg)
        self.buttonStartCapture.setEnabled(False)
        QTimer.singleShot(1000, lambda : self.buttonStartCapture.setEnabled(not self.captureActive))
        

    def stopCapture(self):
        msg = WindowMessage(MsgType.STOPSERIAL)
        self.displayPipe.send(msg)
        

    def pauseCapture(self, paused=None, pause=None):
        if pause == None:
            pause = not self.capturePaused
        self.buttonPauseCapture.setEnabled(True)
        if pause: 
            self.buttonPauseCapture.setText("Continue")
            self.buttonPauseCapture.setChecked(True)
            self.capturePaused = True
        else:
            self.buttonPauseCapture.setText("Pause")
            self.capturePaused = False
            self.buttonPauseCapture.setChecked(False)
        


        
