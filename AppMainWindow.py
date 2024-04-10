from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QFrame, QGroupBox, QRadioButton
from PyQt6.QtCore import QTimer, QSettings
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
        self.settings = QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope, "shiggytech", "infrasound")
        self.settings.setFallbacksEnabled(False)
        self.settings.setValue("Settings/Version", "0.1")
        self.xs = [0] # sample in time
        self.ys = [0] # measured value in Pa
        self.F = float(self.settings.value("Data/Samplerate", 50)) # Samplerate F
        self.N = float(self.settings.value("Data/SamplesBuffered", 500)) # number of samples retained in buffer
        self.useSPL = True # Output fft data in dBSPL instead of Pa
        self.captureActive = False
        self.capturePaused = False
        self.serialPorts = list()
        self.setWindowTitle("Infrasound Analysis")
        mainWidget = QWidget()
        vLayout = QVBoxLayout()
        hTopLayout = QHBoxLayout()
        settingsLayout = QGridLayout()
        settingsSeparator1 = QFrame()
        settingsSeparator2 = QFrame()
        self.plotGraph = pg.PlotWidget()
        self.fftGraph = pg.PlotWidget()
        buttonReloadSerial = QPushButton("Reload")
        self.comboSelectSerial = QComboBox()
        self.buttonStartCapture = QPushButton("Start")
        self.buttonStopCapture = QPushButton("Stop")
        self.buttonPauseCapture = QPushButton("Pause")
        spinSamplerate = QDoubleSpinBox()
        spinCaptureTime = QDoubleSpinBox()
        groupBoxUnit = QGroupBox("Unit")
        self.radioUnitSpl = QRadioButton("dbₛₚₗ")
        self.radioUnitLin = QRadioButton("Pa")
        unitLayout = QHBoxLayout()



        # Set Top Layout with 2 Graphs
        vLayout.addLayout(hTopLayout)
        hTopLayout.addWidget(self.plotGraph)
        hTopLayout.addWidget(self.fftGraph)
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
        buttonReloadSerial.clicked.connect(self.reloadSerial)
        self.comboSelectSerial.setEditable(True)
        self.comboSelectSerial.setCurrentText(self.settings.value("Serial/Port", ""))
        self.buttonStartCapture.clicked.connect(self.startCapture)
        self.buttonStopCapture.clicked.connect(self.stopCapture)
        self.buttonStopCapture.setEnabled(False)
        self.buttonPauseCapture.clicked.connect(self.pauseCapture)
        self.buttonPauseCapture.setCheckable(True)
        self.buttonPauseCapture.setEnabled(False)
        # Settings Column 3: Display Settings
        settingsLayout.addWidget(QLabel("Data Display"),0,3)
        settingsLayout.addWidget(spinSamplerate,1,3)
        settingsLayout.addWidget(spinCaptureTime,2,3)
        settingsLayout.addWidget(groupBoxUnit,3,3)
        
        spinSamplerate.setValue(self.F)
        spinSamplerate.setMinimum(0.01)
        spinSamplerate.setMaximum(100000)
        spinSamplerate.setSingleStep(1.0)
        spinSamplerate.setStepType(QDoubleSpinBox.StepType.AdaptiveDecimalStepType)
        spinSamplerate.setSuffix(" S/s")
        spinSamplerate.setToolTip("Samplerate in Samples per Second")
        spinSamplerate.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        spinSamplerate.setAccelerated(True)
        spinSamplerate.setKeyboardTracking(False)
        spinSamplerate.valueChanged.connect(self.setSamplerate)
        spinCaptureTime.setValue(self.N/self.F)
        spinCaptureTime.setMinimum(1)
        spinCaptureTime.setMaximum(86400)
        spinCaptureTime.setSingleStep(10.0)
        spinCaptureTime.setStepType(QDoubleSpinBox.StepType.AdaptiveDecimalStepType)
        spinCaptureTime.setSuffix(" s")
        spinCaptureTime.setToolTip("Capturetime in Seconds")
        spinCaptureTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        spinCaptureTime.setAccelerated(True)
        spinCaptureTime.setKeyboardTracking(False)
        spinCaptureTime.valueChanged.connect(self.setCaptureTime)
        unitLayout.addWidget(self.radioUnitSpl)
        unitLayout.addWidget(self.radioUnitLin)
        unitLayout.addStretch()
        groupBoxUnit.setLayout(unitLayout)
        if self.settings.value("Display/Unit") == "Pa":
            self.radioUnitLin.setChecked(True)
            self.useSPL = False
        elif self.settings.value("Display/Unit") == "dbSPL":
            self.radioUnitSpl.setChecked(True)
            self.useSPL = True
        else:
            self.radioUnitSpl.setChecked(True)
        self.radioUnitLin.clicked.connect(self.selectUnit)
        self.radioUnitSpl.clicked.connect(self.selectUnit)


        # Settings Column 5: Saving Data
        settingsLayout.addWidget(QLabel("Data Saving"),0,5)
        settingsLayout.addWidget(QPushButton("Save (png)"),3,5)
        settingsLayout.addWidget(QPushButton("Save (csv)"),4,5)
        

        # Add Layout to Main Window
        mainWidget.setLayout(vLayout)
        self.setCentralWidget(mainWidget)
        self.lineT = self.plotGraph.plot(self.xs,self.ys)
        self.plotGraph.getPlotItem().getAxis("bottom").setLabel("time ", units="s")
        self.plotGraph.getPlotItem().getAxis("left").setLabel("Pressure ", units="Pa")
        self.lineF = self.fftGraph.plot(self.xs,self.ys)
        self.fftGraph.getPlotItem().getAxis("bottom").setLabel("Frequency ", units="Hz")
        if self.useSPL:
            self.fftGraph.getPlotItem().getAxis("left").setLabel("Pressure ", units="dB_SPL")
        else:
            self.fftGraph.getPlotItem().getAxis("left").setLabel("Pressure ", units="Pa")
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
        while(len(xs)>self.N):
            xs.pop(0)
            ys.pop(0)

    def updateData(self):
         self.q2Vars(self.q, self.xs, self.ys)

    def updateTPlot(self):
        xs = np.array(self.xs)*1/self.F
        ys = np.array(self.ys)
        self.lineT.setData(xs, ys)

    def updateFPlot(self):
        T = 1/self.F 
        N = len(self.ys)
        if N > 1:
            if N%2:
                N += 1
            ys = np.array(self.ys)
            yf = fft(ys, n=N, norm="forward")
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
        port = self.comboSelectSerial.currentText()
        msg = WindowMessage(MsgType.STARTSERIAL, SerialPortSettings(port=port, baudrate=38400))
        self.displayPipe.send(msg)
        self.buttonStartCapture.setEnabled(False)
        self.settings.setValue("Serial/Port", port)
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

    def setSamplerate(self, samplerate):
        oldSampleRate = self.F
        if samplerate > 0:
            self.F = float(samplerate)
        else:
            self.F = 0.01
        self.setCaptureTime(self.N*oldSampleRate)
        self.settings.setValue("Data/Samplerate", self.F)

    def setCaptureTime(self, captureTime):
        if captureTime > 0:
            self.N = captureTime*self.F
        else:
            self.N = self.F
        self.settings.setValue("Data/SamplesBuffered", self.N)
    
    def selectUnit(self, active):
        if active:
            if self.radioUnitLin.isChecked():
                self.useSPL = False
                self.fftGraph.getPlotItem().getAxis("left").setLabel("Pressure ", units="Pa")
                self.settings.setValue("Display/Unit", "Pa")
            elif self.radioUnitSpl.isChecked():
                self.useSPL = True
                self.fftGraph.getPlotItem().getAxis("left").setLabel("Pressure ", units="dbₛₚₗ")
                self.settings.setValue("Display/Unit", "dbSPL")


        
