from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout
from multiprocessing import Queue
import pyqtgraph as pg
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, q:Queue) -> None:
        super().__init__()
        self.setWindowTitle("Infraschall Analyse")
        mainWidget = QWidget()
        vLayout = QVBoxLayout()
        hTopLayout = QHBoxLayout()      
        plot_graph = pg.PlotWidget()
        fft_graph = pg.PlotWidget()
        vLayout.addLayout(hTopLayout)
        hTopLayout.addWidget(plot_graph)
        hTopLayout.addWidget(fft_graph)
        mainWidget.setLayout(vLayout)
        self.setCentralWidget(mainWidget)
        self.xs = [0]
        self.ys = [0]
        self.line = plot_graph.plot(
            self.xs,
            self.ys
        )
        self.q = q
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()
        
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

    def update_plot(self):
         self.q2Vars(self.q, self.xs, self.ys)
         self.line.setData(self.xs, self.ys)
