from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QFrame, QGroupBox, QRadioButton, QFileDialog, QCheckBox, QMessageBox
from PyQt6.QtCore import QTimer, QSettings, QSize, QRegularExpression
from PyQt6.QtGui import QImage, QGuiApplication, QRegularExpressionValidator, QValidator, QTransform
from multiprocessing import Queue, connection
import pyqtgraph as pg
import pyqtgraph.exporters
from pyqtgraph.Qt import QtGui
from scipy.fft import fft,fftfreq
from scipy.signal import ShortTimeFFT, windows
from scipy.io import wavfile
import numpy as np
import serial.tools.list_ports
from enum import Enum, auto
from typing import NamedTuple, Tuple
from datetime import datetime
import math

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
        if self.settings.value("Settings/Version", 0.0, float) != 0.2:
            self.updateSettings()
        self.xs = [0] # sample in time
        self.ys = [0] # measured value in Pa
        self.F = float(self.settings.value("Data/Samplerate", 50)) # Samplerate F
        self.N = int(self.settings.value("Data/SamplesBuffered", 500)) # number of samples retained in buffer
        self.useSPL = self.settings.value("Display/Unit", "dbSPL") == "dbSPL" # Output fft data in dBSPL instead of Pa
        self.fPlotType = 'spectogram' # fft, spectogram
        self.fftYLimitAuto = self.settings.value("Display/FftYLimit", "auto").lower() == "auto" # calculate upper limit from current values
        self.fftYLimit = 90 if self.fftYLimitAuto else float(self.settings.value("Display/FftYLimit",90)) # upper limit of the fft y axis
        self.fftRange = ["min", "max"]
        self.fftMaxRange = self.settings.value("Analysis/FftMaxRange", [0.1, 'max']) # range to search for fft maximum value
        if len(self.fftMaxRange) < 2:
            self.fftMaxRange = [0.1, 'max']
            self.settings.setValue("Analysis/FftMaxRange", self.fftMaxRange)
        else:
            self.fftMaxRange = [x if x == "min" or x == "max" else float(x) if self._isFloat(x) else "min" if i == 0 else "max" for i, x in enumerate(self.fftMaxRange[0:2])] # limit list to the first 2 values. Use them if they are min, max or float. Otherwise set min if first value or max if second value. 
        self.volumeRange = [round(max(0, self.N-0.3*self.F)), "max"]
        self.spectFreqResolution = 0.1
        self.spectTimeResolution = 1
        self.captureActive = False # Is the input port active
        self.capturePaused = False # Is the receiving of data paused
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
        self.spectGraph = pg.PlotItem()
        self.spectImg = pg.ImageItem()
        buttonReloadSerial = QPushButton("Reload")
        self.comboSelectSerial = QComboBox()
        self.buttonStartCapture = QPushButton("Start")
        self.buttonStopCapture = QPushButton("Stop")
        self.buttonPauseCapture = QPushButton("Pause")
        self.buttonLoadData = QPushButton("Open File...")
        self.spinSamplerate = QDoubleSpinBox()
        self.spinCaptureTime = QDoubleSpinBox()
        self.capSampleLabel = QLabel()
        self.capResolutionLabel = QLabel()
        groupBoxCaptureInfo = QGroupBox("Capture Info")
        captureInfoLayout = QVBoxLayout()
        groupBoxFftTime = QGroupBox("FFT Range")
        fftTimeLayout = QHBoxLayout()
        self.spinFftStartTime = self.CustomDoubleSpinBox()
        self.spinFftEndTime = self.CustomDoubleSpinBox()
        groupBoxFftMaxRange = QGroupBox("FFT Analysis Search Range")
        fftMaxRangeLayout = QHBoxLayout()
        self.spinFftMaxStart = self.CustomDoubleSpinBox()
        self.spinFftMaxEnd = self.CustomDoubleSpinBox()
        groupBoxVolumeTime = QGroupBox("Volume Range")
        volumeTimeLayout = QHBoxLayout()
        self.spinVolumeStartTime = self.CustomDoubleSpinBox()
        buttonExportPng = QPushButton("Save (Image)")
        buttonExportCsv = QPushButton("Save (Text)")
        buttonExportWav = QPushButton("Save (Audio)")
        fftYLimitLayout = QHBoxLayout()
        self.spinFftYLimit = QDoubleSpinBox()
        cbFftYLimitAuto = QCheckBox("auto")
        



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
        self.spectGraph.addItem(self.spectImg)


        # Set Bot Layout with a settings Grid
        vLayout.addLayout(settingsLayout)
        settingsLayout.setHorizontalSpacing(20)

        settingsLayout.addWidget(settingsSeparator1,1, 2, 6, 1)
        settingsLayout.addWidget(settingsSeparator2,1, 4, 6, 1)
        settingsLayout.addWidget(settingsSeparator3,1, 6, 6, 1)
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
        settingsLayout.addWidget(self.buttonLoadData,3,1)
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
        self.buttonLoadData.clicked.connect(self.loadData)

        # Settings Column 3: Data Settings
        # Settings column for setting input parameters like capture time and samplerate
        settingsLayout.addWidget(QLabel("Data Settings"),0,3)
        settingsLayout.addWidget(self.spinSamplerate,1,3)
        settingsLayout.addWidget(self.spinCaptureTime,2,3)
        settingsLayout.addWidget(groupBoxCaptureInfo,4,3,3,1)
        
        self.spinSamplerate.setMinimum(0.01)
        self.spinSamplerate.setMaximum(100000)
        self.spinSamplerate.setValue(self.F)
        self.spinSamplerate.setSingleStep(1.0)
        self.spinSamplerate.setStepType(QDoubleSpinBox.StepType.AdaptiveDecimalStepType)
        self.spinSamplerate.setSuffix(" S/s")
        self.spinSamplerate.setToolTip("Samplerate in Samples per Second")
        self.spinSamplerate.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinSamplerate.setAccelerated(True)
        self.spinSamplerate.setKeyboardTracking(False)
        self.spinSamplerate.valueChanged.connect(self.setSamplerate)
        self.spinCaptureTime.setMinimum(1)
        self.spinCaptureTime.setMaximum(86400)
        self.spinCaptureTime.setValue(self.N/self.F)
        self.spinCaptureTime.setSingleStep(10.0)
        self.spinCaptureTime.setStepType(QDoubleSpinBox.StepType.AdaptiveDecimalStepType)
        self.spinCaptureTime.setSuffix(" s")
        self.spinCaptureTime.setToolTip("Capturetime in Seconds")
        self.spinCaptureTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinCaptureTime.setAccelerated(True)
        self.spinCaptureTime.setKeyboardTracking(False)
        self.spinCaptureTime.valueChanged.connect(self.setCaptureTime)
        captureInfoLayout.addWidget(self.capSampleLabel)
        captureInfoLayout.addWidget(self.capResolutionLabel)
        captureInfoLayout.addStretch()
        groupBoxCaptureInfo.setLayout(captureInfoLayout)
        self.updateCaptureInfo()

        # Settings Column 5: Analysis Settings
        # setting column for setting parameters used for the analysis like fft area
        settingsLayout.addWidget(QLabel("Analysis Settings"),0,5)
        settingsLayout.addWidget(groupBoxFftTime,1,5,2,1)
        settingsLayout.addWidget(groupBoxFftMaxRange,3,5,2,1)
        settingsLayout.addWidget(groupBoxVolumeTime,5,5,2,1)
        
        fftTimeLayout.addWidget(self.spinFftStartTime)
        fftTimeLayout.addWidget(self.spinFftEndTime)
#        fftTimeLayout.addStretch()
        groupBoxFftTime.setLayout(fftTimeLayout)

        self.spinFftStartTime.setMinimum(0)
        self.spinFftStartTime.setMaximum(self.N/self.F)
        self.spinFftStartTime.setValue(0)
        self.spinFftStartTime.setSingleStep(1/self.F)
        self.spinFftStartTime.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinFftStartTime.setSuffix(" s")
        self.spinFftStartTime.setToolTip("Starting time for fft calculation")
        self.spinFftStartTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinFftStartTime.setAccelerated(True)
        self.spinFftStartTime.setKeyboardTracking(False)
        self.spinFftStartTime.valueChanged.connect(lambda t: self.setFftRange(start=t))
        self.spinFftEndTime.setMinimum(0)
        self.spinFftEndTime.setMaximum(self.N/self.F)
        self.spinFftEndTime.setValue(self.N/self.F)
        self.spinFftEndTime.setSingleStep(1/self.F)
        self.spinFftEndTime.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinFftEndTime.setSuffix(" s")
        self.spinFftEndTime.setToolTip("Stopping time for fft calculation")
        self.spinFftEndTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinFftEndTime.setAccelerated(True)
        self.spinFftEndTime.setKeyboardTracking(False)
        self.spinFftEndTime.valueChanged.connect(lambda t: self.setFftRange(end=t))
        
        fftMaxRangeLayout.addWidget(self.spinFftMaxStart)
        fftMaxRangeLayout.addWidget(self.spinFftMaxEnd)

        groupBoxFftMaxRange.setLayout(fftMaxRangeLayout)

        self.spinFftMaxStart.setMinimum(0)
        self.spinFftMaxStart.setMaximum(round(self.F/2, 2))
        self.spinFftMaxStart.setValue(self.fftMaxRange[0])
        self.spinFftMaxStart.setSingleStep(0.01)
        self.spinFftMaxStart.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinFftMaxStart.setSuffix(" Hz")
        self.spinFftMaxStart.setToolTip("Starting frequency for fft maximum search")
        self.spinFftMaxStart.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinFftMaxStart.setAccelerated(True)
        self.spinFftMaxStart.setKeyboardTracking(False)
        self.spinFftMaxStart.valueChanged.connect(lambda f: self.setFftMaxRange(start=f))
        self.spinFftMaxEnd.setMinimum(0)
        self.spinFftMaxEnd.setMaximum(round(self.F/2, 2))
        self.spinFftMaxEnd.setValue(self.fftMaxRange[1])
        self.spinFftMaxEnd.setSingleStep(0.01)
        self.spinFftMaxEnd.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinFftMaxEnd.setSuffix(" Hz")
        self.spinFftMaxEnd.setToolTip("Stopping frequendy for fft maximum search")
        self.spinFftMaxEnd.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinFftMaxEnd.setAccelerated(True)
        self.spinFftMaxEnd.setKeyboardTracking(False)
        self.spinFftMaxEnd.valueChanged.connect(lambda f: self.setFftMaxRange(end=f))

        volumeTimeLayout.addWidget(self.spinVolumeStartTime)
        groupBoxVolumeTime.setLayout(volumeTimeLayout)

        self.spinVolumeStartTime.setMinimum(0)
        self.spinVolumeStartTime.setMaximum(self.N/self.F)
        self.spinVolumeStartTime.setValue(self.volumeRange[0]/self.F)
        self.spinVolumeStartTime.setSingleStep(1/self.F)
        self.spinVolumeStartTime.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinVolumeStartTime.setSuffix(" s")
        self.spinVolumeStartTime.setToolTip("Starting time for volume calculation")
        self.spinVolumeStartTime.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinVolumeStartTime.setAccelerated(True)
        self.spinVolumeStartTime.setKeyboardTracking(False)
        self.spinVolumeStartTime.valueChanged.connect(lambda t: self.setVolumeRange(start=t))

        # Settings Column 7: Saving Data
        # settings column for exporting data 
        settingsLayout.addWidget(QLabel("Display and Data Saving"),0,7)
        fftYLimitLayout.addWidget(QLabel("FFT y-limit: "))
        fftYLimitLayout.addWidget(self.spinFftYLimit)
        fftYLimitLayout.addWidget(cbFftYLimitAuto)
        settingsLayout.addLayout(fftYLimitLayout,1,7)
        settingsLayout.addWidget(buttonExportPng,4,7)
        settingsLayout.addWidget(buttonExportCsv,5,7)
        settingsLayout.addWidget(buttonExportWav,6,7)
        buttonExportPng.clicked.connect(lambda :self.exportData(png=True))
        buttonExportCsv.clicked.connect(lambda :self.exportData(csv=True))
        buttonExportWav.clicked.connect(lambda :self.exportData(wav=True))
        
        self.spinFftYLimit.setMinimum(0)
        self.spinFftYLimit.setMaximum(122)
        self.spinFftYLimit.setValue(self.fftYLimit)
        self.spinFftYLimit.setSingleStep(1.0)
        self.spinFftYLimit.setStepType(QDoubleSpinBox.StepType.DefaultStepType)
        self.spinFftYLimit.setSuffix(" dbₛₚₗ")
        self.spinFftYLimit.setToolTip("Upper Limit for the FFT Y axis in dbₛₚₗ")
        self.spinFftYLimit.setCorrectionMode(QDoubleSpinBox.CorrectionMode.CorrectToNearestValue)
        self.spinFftYLimit.setAccelerated(True)
        self.spinFftYLimit.setKeyboardTracking(False)
        self.spinFftYLimit.setEnabled(False)
        self.spinFftYLimit.valueChanged.connect(self.setFftYLimit)
        cbFftYLimitAuto.setChecked(True)
        cbFftYLimitAuto.clicked.connect(self.setFftYLimitAuto)
        
        # Add Layout to Main Window
        mainWidget.setLayout(vLayout)
        self.setCentralWidget(mainWidget)
        
        # draw the initial plots
        plotPen = pg.mkPen(color=(10, 10, 10), width=2)
        fftBorderPen = pg.mkPen(color=(240, 10, 10), width = 3, style=QtCore.Qt.PenStyle.DashLine)
        fftMaxBorderPen = pg.mkPen(color=(240, 10, 10, 128), width = 3, style=QtCore.Qt.PenStyle.DashLine) # Pen to style the lines showing the fft maximum search range
        volumeBorderPen = pg.mkPen(color=(10, 10, 240), width = 3, style=QtCore.Qt.PenStyle.DashLine)
        
        evPen = pg.mkPen(color=(240, 10, 10), width=2) # marker Pen for extreme values
        evBrush = pg.mkBrush(None) # transparent Brush for extreme values
        self.plotGraph.addLegend(offset=(-2, 2), pen=plotPen, brush=pg.mkBrush(color=(255, 255, 255, 210)), labelTextColor=plotPen.color())
        self.lineT = self.plotGraph.plot(self.xs,self.ys, pen=plotPen)
        self.lineTFftStart = self.plotGraph.plot([0], [0], pen=fftBorderPen)
        self.lineTFftEnd = self.plotGraph.plot([0], [0], pen=fftBorderPen)
        self.lineTVolumeStart = self.plotGraph.plot([0], [0], pen=volumeBorderPen, name="volume")
        self.plotGraph.getAxis("bottom").setLabel("time ", units="s")
        self.plotGraph.getAxis("left").setLabel("Pressure ", units="Pa")
        self.fftGraph.addLegend(offset=(-2, 2), pen=plotPen, brush=pg.mkBrush(color=(255, 255, 255, 210)), labelTextColor=plotPen.color())
        self.lineF = self.fftGraph.plot(self.xs,self.ys, pen=plotPen)
        self.lineFMaxStart = self.fftGraph.plot([0], [0], pen=fftMaxBorderPen) # draw default lower fft maximum search limit
        self.lineFMaxEnd = self.fftGraph.plot([0], [0], pen=fftMaxBorderPen) # draw default upper fft maximum search limit
        self.lineFMax = self.fftGraph.plot(x=[12], y=[10], symbol="t1", pen=pg.mkPen(None), symbolPen=evPen, symbolBrush=evBrush, name="maximum") # use initial values that are visible
        self.fftGraph.getAxis("bottom").setLabel("Frequency ", units="Hz")
        if self.useSPL:
            self.fftGraph.getAxis("left").setLabel("Pressure ", units="dB_SPL")
        else:
            self.fftGraph.getAxis("left").setLabel("Pressure ", units="Pa")
        self.spectColors = pg.colormap.get(name="plasma")
        self.spectImg.setLookupTable(self.spectColors.getLookupTable())
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
        fftStart = 0 if self.fftRange[0] == "min" else np.min([self.N,len(xs)])-1 if self.fftRange[0] == "max" else np.min([self.fftRange[0], len(xs)])-1
        fftEnd = 0 if self.fftRange[1] == "min" else np.min([self.N,len(xs)])-1 if self.fftRange[1] == "max" else np.min([self.fftRange[1], len(xs)])-1
        self.lineTFftStart.setData([xs[fftStart], xs[fftStart]],[-100, 100])
        self.lineTFftEnd.setData([xs[fftEnd], xs[fftEnd]],[-100, 100])

        volumeStart = 0 if self.volumeRange[0] == "min" else np.min([self.N,len(xs)])-1 if self.volumeRange[0] == "max" else np.min([self.volumeRange[0], len(xs)])-1
        volumeEnd = 0 if self.volumeRange[1] == "min" else np.min([self.N,len(xs)])-1 if self.volumeRange[1] == "max" else np.min([self.volumeRange[1], len(xs)])-1
        self.lineTVolumeStart.setData([xs[volumeStart], xs[volumeStart]],[-100, 100])
        ysV = ys[volumeStart:volumeEnd+1]
        if len(ysV) >= 1:
            volumeRms = math.dist(ysV,[0]*len(ysV))/math.sqrt(len(ysV))
            volumeSpl = -math.inf
            if volumeRms != 0:
                volumeSpl = self.lin2dbSPL(volumeRms)
            self.plotGraph.addLegend().getLabel(self.lineTVolumeStart).setText("volume: {volume:.2f} dbₛₚₗ".format(volume=volumeSpl))
        else:
            self.plotGraph.addLegend().getLabel(self.lineTVolumeStart).setText("no Data!")
        self.plotGraph.setXRange(min=xs[0], max=xs[-1], padding=0)
        self.plotGraph.setYRange(min=np.min(ys), max=np.max(ys), padding=0.1)
        self.updateCaptureInfo()

    def updateFPlot(self):
        T = 1/self.F 
        if self.fPlotType == "fft":
            fftStart = 0 if self.fftRange[0] == "min" else np.min([self.N,len(self.ys)])-1 if self.fftRange[0] == "max" else np.min([self.fftRange[0], len(self.ys)])-1
            fftEnd = 0 if self.fftRange[1] == "min" else np.min([self.N,len(self.ys)])-1 if self.fftRange[1] == "max" else np.min([self.fftRange[1], len(self.ys)])-1
            N = len(self.ys[fftStart:(fftEnd+1)])
            if N > 1:
                # Enough values to calculate an fft are captured and selected. 
                if N%2:
                    N += 1
                ys = np.array(self.ys[fftStart:(fftEnd+1)])
                yf = fft(ys, n=N, norm="forward")
                xf = fftfreq(N, T)[:N//2]
                yfs = abs(yf[0:N//2])
                yfs[1:-1] *= 2.0 

                fftMaxStart = 0 if self.fftMaxRange[0] == "min" else len(xf)-1 if self.fftMaxRange[0] == "max" else np.min([round(self.fftMaxRange[0]*N/self.F), len(xf)])-1 # index to start searching for maximum fft value
                fftMaxEnd = 0 if self.fftMaxRange[1] == "min" else len(xf)-1 if self.fftMaxRange[1] == "max" else np.min([round(self.fftMaxRange[1]*N/self.F), len(xf)])-1 # index to stop searching for maximum fft value
                fftYRange = [0, 0]
                if self.useSPL:
                    yfsLog = self.lin2dbSPL(yfs)  
                    self.lineF.setData(xf, yfsLog)
                    if fftMaxEnd > fftMaxStart:
                        yfsMaxIdx = np.argmax(yfsLog[fftMaxStart:fftMaxEnd]) + fftMaxStart # search max between fftMaxStart and fftMaxEnd and fix offset to xf by adding fftMaxStart
                        self.lineFMax.setData([xf[yfsMaxIdx]], [yfsLog[yfsMaxIdx]])
                        self.fftGraph.addLegend().getLabel(self.lineFMax).setText("max: {maxval:.2f} dbₛₚₗ @ {freq:.1f} Hz".format(maxval=yfsLog[yfsMaxIdx], freq=xf[yfsMaxIdx]))
                    else:
                        self.lineFMax.setData([], [])
                        self.fftGraph.addLegend().getLabel(self.lineFMax).setText("invalid max. search limits")
                    if self.fftYLimitAuto:
                        fftYRange[1] = np.max(yfsLog)
                    else:
                        fftYRange[1] = self.fftYLimit
                else:
                    self.lineF.setData(xf, yfs)
                    if fftMaxEnd > fftMaxStart:
                        yfsMaxIdx = np.argmax(yfs[fftMaxStart:fftMaxEnd]) + fftMaxStart # search max between fftMaxStart and fftMaxEnd and fix offset to xf by adding fftMaxStart
                        self.lineFMax.setData([xf[yfsMaxIdx]], [yfs[yfsMaxIdx]])
                        self.fftGraph.addLegend().getLabel(self.lineFMax).setText("max: {maxval:.2f} Pa @ {freq:.1f} Hz".format(maxval=yfs[yfsMaxIdx], freq=xf[yfsMaxIdx]))
                    else:
                        self.lineFMax.setData([], [])
                        self.fftGraph.addLegend().getLabel(self.lineFMax).setText("invalid max. search limits")
                    if self.fftYLimitAuto:
                        fftYRange[1] = np.max(yfs)
                    else:
                        fftYRange[1] = self.fftYLimit
                self.fftGraph.setYRange(min=0, max=fftYRange[1], padding=0.1)
                self.fftGraph.setXRange(min=xf[0], max=xf[-1], padding=0)
                
                self.lineFMaxStart.setData([xf[fftMaxStart], xf[fftMaxStart]],self.fftGraph.getViewBox().viewRange()[1])
                self.lineFMaxEnd.setData([xf[fftMaxEnd], xf[fftMaxEnd]],self.fftGraph.getViewBox().viewRange()[1])
            else: 
                # Not enough data captured or selected. 
                # Blank the fft graph and set info text in legend.
                # This path is also taken if fftStart >= fftEnd
                self.lineF.setData([], [])
                self.lineFMax.setData([], [])
                self.fftGraph.addLegend().getLabel(self.lineFMax).setText("no Data!")
        else:
            # spectogram visualization
            currentPlot = self.plotArea.getItem(0, 1)
            if currentPlot is not self.spectGraph:
                if currentPlot is not None:
                    self.plotArea.removeItem(currentPlot)
                self.plotArea.addItem(self.spectGraph, row=0, col=1)
            N = len(self.ys)
            if N > 1:
                ys = np.array(self.ys)
                winLength = round(self.F/self.spectFreqResolution)
                win = windows.gaussian(winLength, std=winLength*0.3, sym=True)
                sft = ShortTimeFFT(win=win, hop=round(self.spectTimeResolution*self.F), fs=self.F, fft_mode="onesided", scale_to="psd")
                spect = sft.spectrogram(ys, detr=None, padding="zeros")
                self.spectImg.setImage(spect.T, autoLevels=True)
                tr = QTransform()
                times = sft.extent(N)[:2]
                freqs = sft.extent(N)[2:]
                dt = (times[1] - times[0])/spect.shape[1]
                df = (freqs[1] - freqs[0])/spect.shape[0]
                tr.translate((times[0]+self.xs[0]/self.F), freqs[0])
                tr.scale(dt, df)
                self.spectImg.setTransform(tr)
                pass
            
    
    def updateComms(self):
        while self.displayPipe.poll():
            msg = self.displayPipe.recv()
            try:
                if msg.type == MsgType.SERIALSTARTED:
                    self.captureActive = True
                    self.buttonStopCapture.setEnabled(True)
                    self.pauseCapture(pause=False)
                    self.buttonLoadData.setEnabled(False)
                elif msg.type == MsgType.SERIALSTOPPED:
                    self.captureActive = False
                    self.buttonStartCapture.setEnabled(True)
                    self.buttonStopCapture.setEnabled(False)
                    self.buttonPauseCapture.setEnabled(False)
                    self.buttonLoadData.setEnabled(True)
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
        self.spinFftStartTime.setSingleStep(1/self.F)
        self.spinFftEndTime.setSingleStep(1/self.F)
        self.spinVolumeStartTime.setSingleStep(1/self.F)
        self.spinFftMaxStart.setMaximum(round(self.F/2, 2))
        self.spinFftMaxEnd.setMaximum(round(self.F/2, 2))
        # update fft max range spinners with new maximum value
        newFftMaxRangeStart = 0 if self.fftMaxRange[0] == "min" else self.F/2 if self.fftMaxRange[0] == "max" else self.fftMaxRange[0]
        newFftMaxRangeEnd = 0 if self.fftMaxRange[1] == "min" else self.F/2 if self.fftMaxRange[1] == "max" else self.fftMaxRange[1]
        self.spinFftMaxStart.setValue(newFftMaxRangeStart)
        self.spinFftMaxEnd.setValue(newFftMaxRangeEnd)      

    def setCaptureTime(self, captureTime):
        if captureTime > 0:
            self.N = math.ceil(captureTime*self.F)
        else:
            self.N = math.ceil(self.F)
        self.settings.setValue("Data/SamplesBuffered", self.N)
        newFftStartValue = 0
        if self.fftRange[0] == "min":
            pass
        elif self.fftRange[0] == "max":
            newFftStartValue = self.N/self.F
        elif self.fftRange[0] >= self.N:
            newFftStartValue = (self.N-1)/self.F
        else:
            newFftStartValue = self.fftRange[0]/self.F
        newFftEndValue = 0
        if self.fftRange[1] == "min":
            pass
        elif self.fftRange[1] == "max":
            newFftEndValue = self.N/self.F
        elif self.fftRange[1] >= self.N:
            newFftEndValue = (self.N-1)/self.F
        else:
            newFftEndValue = self.fftRange[1]/self.F
        newVolumeStartValue = 0
        if self.volumeRange[0] == "min":
            pass
        elif self.volumeRange[0] == "max":
            newVolumeStartValue = self.N/self.F
        elif self.volumeRange[0] >= self.N:
            newVolumeStartValue = (self.N-1)/self.F
        else:
            newVolumeStartValue = self.volumeRange[0]/self.F
        self.spinFftStartTime.setMaximum(self.N/self.F)
        self.spinFftEndTime.setMaximum(self.N/self.F)
        self.spinVolumeStartTime.setMaximum(self.N/self.F)
        self.spinFftStartTime.setValue(newFftStartValue)
        self.spinFftEndTime.setValue(newFftEndValue)
        self.spinVolumeStartTime.setValue(newVolumeStartValue)
    
    def exportData(self, png=False, csv=False, wav=False):
        defaultName = datetime.now().strftime("%Y-%m-%d-%H%M%S")
        if png:
            pngExporter = pyqtgraph.exporters.ImageExporter(self.plotArea.scene())
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Image", directory=defaultName+".png", filter="Images (*.png *.jpg)")
            if targetPath:
                img =  QImage(pngExporter.export(toBytes=True))
                img.save(targetPath)
        if csv:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Text", directory=defaultName+".csv", filter="Text (*.csv *.txt)")
            if targetPath:
                xs = np.array(self.xs)/self.F
                ys = np.array(self.ys)
                data = np.transpose((xs, ys))
                hdrString = "Data Recorded and exported with Shiggytech's Infrasound. For More information visit ..."
                np.savetxt(fname=targetPath, X=data, fmt=("%g", "%.5g"), delimiter=";", header=hdrString)
        if wav:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Audio", directory=defaultName+".wav", filter="Audio (*.wav)")
            if targetPath:
                ys = np.array(self.ys)
                scalingFactor = np.iinfo(np.int16).max/np.max(ys)
                data = ys*scalingFactor
                wavfile.write(targetPath, round(self.F*128), data.astype(np.int16))

    def updateCaptureInfo(self):
        N = len(self.ys)
        self.capSampleLabel.setText("max Samples retained: {:.0f}".format(self.N))
        self.capResolutionLabel.setText("max Resolution: {:.3f} Hz".format(self.F/N))
        
    def closeEvent(self, event):
        self.stopCapture()
        msg = WindowMessage(MsgType.STOPWINDOW)
        self.displayPipe.send(msg)
        super().closeEvent(event)
        
    def setFftRange(self, start=None, end=None):
        if start != None:
            if start == 0:
                self.fftRange[0] = "min"
            elif start == self.N/self.F:
                self.fftRange[0] = "max"
            else:
                self.fftRange[0] = round(start*self.F)
        if end != None:
            if end == 0:
                self.fftRange[1] = "min"
            elif end == self.N/self.F:
                self.fftRange[1] = "max"
            else:
                self.fftRange[1] = round(end*self.F)

    def setFftMaxRange(self, start=None, end=None):
        """ Update the FFT maximum‑search frequency range. 
        
        Parameters 
        ---------- 
            start : float or None 
                Lower bound of the search range. Special handling: 
                    - 0 → stored as "min" 
                    - F/2 (rounded to 2 decimals) → stored as "max" 
                    - otherwise the value is rounded to 2 decimals and stored. 
            end : float or None 
                Upper bound of the search range. Same rules as `start`. 
        Notes 
        ----- 
            The function writes the processed values into `self.fftMaxRange`, 
            where index 0 is the start value and index 1 is the end value. 
        """
        if start != None:
            if start == 0:
                self.fftMaxRange[0] = "min"
            elif start == round(self.F/2, 2):
                self.fftMaxRange[0] = "max"
            else:
                self.fftMaxRange[0] = round(start, 2)
        if end != None:
            if end == 0:
                self.fftMaxRange[1] = "min"
            elif end == round(self.F/2, 2):
                self.fftMaxRange[1] = "max"
            else:
                self.fftMaxRange[1] = round(end, 2)
        self.settings.setValue("Analysis/FftMaxRange", self.fftMaxRange)


    def setVolumeRange(self, start=None, end=None):
        if start != None:
            if start == 0:
                self.volumeRange[0] = "min"
            elif start == self.N/self.F:
                self.volumeRange[0] = "max"
            else:
                self.volumeRange[0] = round(start*self.F)
        if end != None:
            if end == 0:
                self.volumeRange[1] = "min"
            elif end == self.N/self.F:
                self.volumeRange[1] = "max"
            else:
                self.volumeRange[1] = round(end*self.F)

    # load Data from an external file
    # the function assumes that the data has been created by this software and expects a compatible format
    def loadData(self):
        if not self.captureActive:
            targetPath, _ = QFileDialog.getOpenFileName(caption="Open Data (Text)", filter="Text (*.csv *.txt)")
            if targetPath:
                data = np.loadtxt(fname=targetPath,delimiter=";")
                xs = np.transpose(data[:,0])
                ys = np.transpose(data[:,1])
                Fs = np.mean(1/np.diff(xs))
                if math.isinf(Fs) or Fs == 0.0:
                    Fs = self.F
                    msg = QMessageBox(self)
                    msg.setWindowTitle("Invalid Samplerate")
                    msg.setText(f"Samplerate estimated from Input data is invalid.\nUsing current value {self.F} S/s instead. ") 
                    msg.setIcon(QMessageBox.Icon.Information) 
                    msg.exec()
                    xs = xs[0] + np.arange(len(xs))/Fs
                self.xs = [int(x) for x in np.round(xs*Fs)]
                self.ys = ys.tolist()
                self.spinSamplerate.setValue(Fs)
                self.spinCaptureTime.setValue(len(xs)/Fs)
#                self.setSamplerate(Fs)
#                self.setCaptureTime(len(xs)/Fs)

    def setFftYLimit(self, yLim):
        if self.useSPL:
            self.fftYLimit = yLim
            self.settings.setValue("Display/FftYLimit", self.fftYLimit)
    
    def setFftYLimitAuto(self, auto):
        if auto:
            self.fftYLimitAuto = True
            self.settings.setValue("Display/FftYLimit", "auto")
            self.spinFftYLimit.setEnabled(False)
        else:
            self.fftYLimitAuto = False
            self.settings.setValue("Display/FftYLimit", self.fftYLimit)
            self.spinFftYLimit.setEnabled(True)

    def updateSettings(self):
        # minor versions update in place or set compatible default values at first update
        if self.settings.value("Settings/Version", 0.0, float) < 0.1:
            self.settings.setValue("Settings/Version", 0.1)
        if self.settings.value("Settings/Version", 0.0, float) < 0.2:
            self.settings.setValue("Settings/Version", 0.2)  
    
    def _isFloat(self, value):
        try:
            float(value)
            return True
        except ValueError:
            return False
    
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
        
        def setValue(self, val):
            if val == "min":
                val = self.minimum()
            elif val == "max":
                val = self.maximum()
            return super().setValue(val)