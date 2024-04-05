from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QGridLayout, QLabel, QComboBox, QDoubleSpinBox, QFrame, QGroupBox
from multiprocessing import Queue
import pyqtgraph as pg
from scipy.fft import fft,fftfreq
import numpy as np

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, q:Queue) -> None:
        super().__init__()
        self.setWindowTitle("Infraschall Analyse")
        mainWidget = QWidget()
        vLayout = QVBoxLayout()
        hTopLayout = QHBoxLayout()
        settingsLayout = QGridLayout()
        settingsSeparator1 = QFrame()
        settingsSeparator2 = QFrame()
        plotGraph = pg.PlotWidget()
        fftGraph = pg.PlotWidget()
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
        settingsLayout.addWidget(QComboBox(),1,1)
        settingsLayout.addWidget(QPushButton("Reload"),2,1)
        settingsLayout.addWidget(QLabel(""),3,1)
        settingsLayout.addWidget(QPushButton("Start"),4,1)
        settingsLayout.addWidget(QPushButton("Pause"),5,1)
        settingsLayout.addWidget(QPushButton("Stop"),6,1)
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
        self.xs = [0]
        self.ys = [0]
        self.useSPL = True
        self.lineT = plotGraph.plot(self.xs,self.ys)
        self.lineF = fftGraph.plot(self.xs,self.ys)
        self.q = q

        # Setup Timers for periodic UI and Data Update
        self.updDataTimer = QtCore.QTimer()
        self.updDataTimer.setInterval(50)
        self.updDataTimer.timeout.connect(self.updateData)
        self.updDataTimer.start()
        self.updPlotTTimer = QtCore.QTimer()
        self.updPlotTTimer.setInterval(100)
        self.updPlotTTimer.timeout.connect(self.updateTPlot)
        self.updPlotTTimer.start()
        self.updPlotFTimer = QtCore.QTimer()
        self.updPlotFTimer.setInterval(200)
        self.updPlotFTimer.timeout.connect(self.updateFPlot)
        self.updPlotFTimer.start()
        
    def q2Vars(self, q, xs, ys):
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

    def updateData(self):
         self.q2Vars(self.q, self.xs, self.ys)
    def updateTPlot(self):
         self.lineT.setData(self.xs, self.ys)
    def updateFPlot(self):
        T = 1/50 # 50 Hz
        N = len(self.ys)
        ys = np.array(self.ys)
        yf = fft(ys)
        xf = fftfreq(N, T)[:N//2]
        self.lineF.setData(xf, 2.0 * abs(yf[0:N//2]))
        
