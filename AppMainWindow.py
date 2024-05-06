from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QFrame, QGroupBox, QRadioButton, QFileDialog
from PyQt6.QtCore import QTimer, QSettings, QSize, QRegularExpression
from PyQt6.QtGui import QImage, QGuiApplication, QRegularExpressionValidator, QValidator
from multiprocessing import Queue, connection
import pyqtgraph as pg
import pyqtgraph.exporters
from scipy.fft import fft,fftfreq
import numpy as np
import serial.tools.list_ports
from enum import Enum, auto
from typing import NamedTuple, Tuple
from datetime import datetime

class MsgType(Enum):
    STARTSERIAL = auto()
    STOPSERIAL = auto()
    SERIALSTARTED = auto()
    SERIALSTOPPED = auto()
    STOPWINDOW = auto()

class WindowMessage(NamedTuple):
    type: MsgType
    payload: object = None

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, p:connection, q:Queue) -> None:
        super().__init__()
        size = QGuiApplication.primaryScreen().availableSize()*0.7
        if size.height() > 720:
            size.setHeight(720)
        if size.width() > 1440:
            size.setWidth(1440)
        self.resize(size)
        self.settings = QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope, "shiggytech", "infrasound")
        self.settings.setFallbacksEnabled(False)
        self.settings.setValue("Settings/Version", "0.1")
        self.xs = [0] # sample in time
        self.ys = [0] # measured value in Pa
        self.F = float(self.settings.value("Data/Samplerate", 50)) # Samplerate F
        self.N = float(self.settings.value("Data/SamplesBuffered", 500)) # number of samples retained in buffer
        self.useSPL = self.settings.value("Display/Unit", "dbSPL") == "dbSPL" # Output fft data in dBSPL instead of Pa
        self.fftRange = ["min", "max"]
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
        settingsSeparator3 = QFrame()
        self.plotArea = pg.GraphicsLayoutWidget()
        self.plotGraph = pg.PlotItem()
        self.fftGraph = pg.PlotItem()
        buttonReloadSerial = QPushButton("Reload")
        self.comboSelectSerial = QComboBox()
        self.buttonStartCapture = QPushButton("Start")
        self.buttonStopCapture = QPushButton("Stop")
        self.buttonPauseCapture = QPushButton("Pause")
        spinSamplerate = QDoubleSpinBox()
        spinCaptureTime = QDoubleSpinBox()
        self.capSampleLabel = QLabel()
        self.capResolutionLabel = QLabel()
        groupBoxCaptureInfo = QGroupBox("Capture Info")
        captureInfoLayout = QVBoxLayout()
        groupBoxFftTime = QGroupBox("FFT Range")
        fftTimeLayout = QHBoxLayout()
        spinFftStartTime = self.CustomDoubleSpinBox()
        spinFftEndTime = self.CustomDoubleSpinBox()
        buttonExportPng = QPushButton("Save (Image)")
        buttonExportCsv = QPushButton("Save (Text)")



        # Set Top Layout with 2 Graphs
        vLayout.addLayout(hTopLayout)
        hTopLayout.addWidget(self.plotArea)
        self.plotArea.addItem(self.plotGraph, row=0, col=0)
        self.plotArea.addItem(self.fftGraph, row=0, col=1)
        self.plotArea.ci.layout.setColumnStretchFactor(0, 1)
        self.plotArea.ci.layout.setColumnStretchFactor(1, 1)
        self.plotArea.setBackground("w")
        self.plotGraph.setMenuEnabled(False)
        self.plotGraph.setMouseEnabled(x=False, y=False)
        self.plotGraph.enableAutoRange(enable=False)
        self.plotGraph.hideButtons()
        self.fftGraph.setMenuEnabled(False)
        self.fftGraph.setMouseEnabled(x=False, y=False)
        self.fftGraph.enableAutoRange(enable=False)
        self.fftGraph.hideButtons()
        


        # Set Bot Layout with a settings Grid
        vLayout.addLayout(settingsLayout)
        settingsLayout.setHorizontalSpacing(20)

        settingsLayout.addWidget(settingsSeparator1,1, 2, 6, 2)
        settingsLayout.addWidget(settingsSeparator2,1, 4, 6, 4)
        settingsLayout.addWidget(settingsSeparator3,1, 6, 6, 6)
        settingsSeparator1.setFrameShape(QFrame.Shape.VLine)
        settingsSeparator2.setFrameShape(QFrame.Shape.VLine)
        settingsSeparator3.setFrameShape(QFrame.Shape.VLine)
        settingsLayout.setColumnStretch(1,1)
        settingsLayout.setColumnStretch(3,1)
        settingsLayout.setColumnStretch(5,1)
        settingsLayout.setColumnStretch(7,1)

        # Settings Column 1: Serial Connection and data Recording
        # Settings Column for selecting the serial connection and cntroling the capture
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
        
        # Settings Column 3: Data Settings
        # Settings column for setting input parameters like capture time and samplerate
        settingsLayout.addWidget(QLabel("Data Settings"),0,3)
        settingsLayout.addWidget(spinSamplerate,1,3)
        settingsLayout.addWidget(spinCaptureTime,2,3)
        settingsLayout.addWidget(groupBoxCaptureInfo,4,3,3,1)
        
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
        captureInfoLayout.addWidget(self.capSampleLabel)
        captureInfoLayout.addWidget(self.capResolutionLabel)
        captureInfoLayout.addStretch()
        groupBoxCaptureInfo.setLayout(captureInfoLayout)
        self.updateCaptureInfo()

        # Settings Column 5: Analysis Settings
        # setting column for setting parameters used for the analysis like fft area
        settingsLayout.addWidget(QLabel("Analysis Settings"),0,5)
        settingsLayout.addWidget(groupBoxFftTime,1,5,2,1)
        
        fftTimeLayout.addWidget(spinFftStartTime)
        fftTimeLayout.addWidget(spinFftEndTime)
        fftTimeLayout.addStretch()
        groupBoxFftTime.setLayout(fftTimeLayout)

        spinFftStartTime.setValue(0)
        spinFftStartTime.setMinimum(0)
        spinFftStartTime.setMaximum(self.N/self.F)
        spinFftStartTime.setSingleStep(1.0)
        spinFftStartTime.setStepType(QDoubleSpinBox.StepType.AdaptiveDecimalStepType)
        spinFftStartTime.setSuffix(" s")
        spinFftStartTime.setToolTip("Starting time for fft calculation")
        spinFftStartTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        spinFftStartTime.setAccelerated(True)
        spinFftStartTime.setKeyboardTracking(False)
#        spinFftStartTime.valueChanged.connect(self.setSamplerate)



        # Settings Column 7: Saving Data
        # settings column for exporting data 
        settingsLayout.addWidget(QLabel("Data Saving"),0,7)
        settingsLayout.addWidget(buttonExportPng,4,7)
        settingsLayout.addWidget(buttonExportCsv,5,7)
        buttonExportPng.clicked.connect(lambda :self.exportData(png=True))
        buttonExportCsv.clicked.connect(lambda :self.exportData(csv=True))
        

        # Add Layout to Main Window
        mainWidget.setLayout(vLayout)
        self.setCentralWidget(mainWidget)
        plotPen = pg.mkPen(color=(10, 10, 10), width=2)
        evPen = pg.mkPen(color=(240, 10, 10), width=2) # marker Pen for extreme values
        evBrush = pg.mkBrush(None) # transparent Brush for extreme values
        self.lineT = self.plotGraph.plot(self.xs,self.ys, pen=plotPen)
        self.plotGraph.getAxis("bottom").setLabel("time ", units="s")
        self.plotGraph.getAxis("left").setLabel("Pressure ", units="Pa")
        self.fftGraph.addLegend(offset=(-2, 2), pen=plotPen, brush=pg.mkBrush(color=(255, 255, 255, 210)), labelTextColor=plotPen.color())
        self.lineF = self.fftGraph.plot(self.xs,self.ys, pen=plotPen)
        self.lineFMax = self.fftGraph.plot(x=[12], y=[10], symbol="t1", pen=pg.mkPen(None), symbolPen=evPen, symbolBrush=evBrush, name="maximum") # use initial values that are visible
        self.fftGraph.getAxis("bottom").setLabel("Frequency ", units="Hz")
        if self.useSPL:
            self.fftGraph.getAxis("left").setLabel("Pressure ", units="dB_SPL")
        else:
            self.fftGraph.getAxis("left").setLabel("Pressure ", units="Pa")
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
        self.plotGraph.setXRange(min=xs[0], max=xs[-1], padding=0)
        self.plotGraph.setYRange(min=np.min(ys), max=np.max(ys), padding=0.1)
        self.updateCaptureInfo()

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
                yfsLog = self.lin2dbSPL(2.0 * abs(yf[0:N//2]))  
                yfsMaxIdx = np.argmax(yfsLog)
                self.lineF.setData(xf, yfsLog)
                self.lineFMax.setData([xf[yfsMaxIdx]], [yfsLog[yfsMaxIdx]])
                self.fftGraph.setYRange(min=np.min(yfsLog), max=np.max(yfsLog), padding=0.1)
                self.fftGraph.addLegend().getLabel(self.lineFMax).setText("max: {maxval:.2f} dbₛₚₗ @ {freq:.1f} Hz".format(maxval=yfsLog[yfsMaxIdx], freq=xf[yfsMaxIdx]))
            else:
                yfs = 2.0 * abs(yf[0:N//2])
                yfsMaxIdx = np.argmax(yfs)
                self.lineF.setData(xf, yfs)
                self.lineFMax.setData([xf[yfsMaxIdx]], [yfs[yfsMaxIdx]])
                self.fftGraph.setYRange(min=np.min(yfs), max=np.max(yfs), padding=0.1)
                self.fftGraph.addLegend().getLabel(self.lineFMax).setText("max: {maxval:.2f} Pa @ {freq:.1f} Hz".format(maxval=yfs[yfsMaxIdx], freq=xf[yfsMaxIdx]))
            self.fftGraph.setXRange(min=xf[0], max=xf[-1], padding=0)
    
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
        self.comboSelectSerial.addItem("virtual")
        if currentIndex == -1 and currentText == "":
            self.comboSelectSerial.setCurrentIndex(0)
        else:
            self.comboSelectSerial.setCurrentText(currentText)
    
    def startCapture(self):
        from Infra import SerialPortSettings
        self.q2Vars(self.q, self.xs, self.ys)
        self.xs = [0]
        self.ys = [0]
        self.updDataTimer.start()
        port = self.comboSelectSerial.currentText()
        msg = WindowMessage(MsgType.STARTSERIAL, SerialPortSettings(port=port, baudrate=38400))
        self.displayPipe.send(msg)
        self.buttonStartCapture.setEnabled(False)
        self.settings.setValue("Serial/Port", port)
        QTimer.singleShot(1000, lambda : self.buttonStartCapture.setEnabled(not self.captureActive))
        

    def stopCapture(self):
        msg = WindowMessage(MsgType.STOPSERIAL)
        self.displayPipe.send(msg)
        self.updDataTimer.stop()
        

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
        self.setCaptureTime(self.N/oldSampleRate)
        self.settings.setValue("Data/Samplerate", self.F)

    def setCaptureTime(self, captureTime):
        if captureTime > 0:
            self.N = captureTime*self.F
        else:
            self.N = self.F
        self.settings.setValue("Data/SamplesBuffered", self.N)
    
    def exportData(self, png=False, csv=False):
        defaultName = datetime.now().strftime("%Y-%m-%d-%H%M%S")
        if png:
            pngExporter = pyqtgraph.exporters.ImageExporter(self.plotArea.scene())
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Image", directory=defaultName+".png", filter="Images (*.png *.jpg)")
            if targetPath:
                img =  QImage(pngExporter.export(toBytes=True))
                img.save(targetPath)
        if csv:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Text", directory=defaultName+".csv", filter="Text (*.csv '.txt)")
            if targetPath:
                xs = np.array(self.xs)/self.F
                ys = np.array(self.ys)
                data = np.transpose((xs, ys))
                hdrString = "Data Recorded and exported with Shiggytech's Infrasound. For More information visit ..."
                np.savetxt(fname=targetPath, X=data, fmt="%.5g", delimiter=";", header=hdrString)

    def updateCaptureInfo(self):
        N = len(self.ys)
        self.capSampleLabel.setText("max Samples retained: {:.0f}".format(self.N))
        self.capResolutionLabel.setText("Resolution: {:.3f} Hz".format(self.F/N))
        
    def closeEvent(self, event):
        self.stopCapture()
        msg = WindowMessage(MsgType.STOPWINDOW)
        self.displayPipe.send(msg)
        super().closeEvent(event)
    
    
    class CustomDoubleSpinBox(QDoubleSpinBox):
        def textFromValue(self, v: float) -> str:
            if v == self.minimum():
                return "min"
            elif v == self.maximum():
                return "max"
            else:
                return super().textFromValue(v)
        
        def valueFromText(self, text: str | None) -> float:
            if text.lower() == "min":
                return self.minimum()
            elif text.lower() == "max":
                return self.maximum()
            else:
                return super().valueFromText(text)
            
        def validate(self, input: str | None, pos: int) -> Tuple[QValidator.State, str, int]:
            validator = QRegularExpressionValidator(QRegularExpression("min|max"))
            result = validator.validate(input.removeprefix(self.prefix()).removesuffix(self.suffix()), pos)
            if result[0] != QValidator.State.Invalid: # could be in the regular expression (Acceptable or Intermediate)
                return result 
            else: # check if it is not Acceptable by default as well
                return super().validate(input, pos)