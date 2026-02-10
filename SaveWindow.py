from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QPushButton, QFileDialog
from PyQt6.QtGui import QImage
from datetime import datetime
from pyqtgraph import GraphicsLayoutWidget
import pyqtgraph.exporters
import numpy as np
from scipy.io import wavfile

class SaveWindow(QDialog):
    def __init__(self, exportArea:GraphicsLayoutWidget, xData:list, yData:list, sampleRate:float, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Save")
        self.defaultName = datetime.now().strftime("%Y-%m-%d-%H%M%S")
        self.exportArea = exportArea
        self.xData = xData
        self.yData = yData
        self.sampleRate = sampleRate
        
        self.buttonLayout = QVBoxLayout()
        buttonExportPng = QPushButton("Save (Image)")
        buttonExportCsv = QPushButton("Save (Text)")
        buttonExportWav = QPushButton("Save (Audio)")
        self.buttonLayout.addWidget(buttonExportPng)
        self.buttonLayout.addWidget(buttonExportCsv)
        self.buttonLayout.addWidget(buttonExportWav)
        buttonExportPng.clicked.connect(lambda :self.exportData(png=True))
        buttonExportCsv.clicked.connect(lambda :self.exportData(csv=True))
        buttonExportWav.clicked.connect(lambda :self.exportData(wav=True))
        
        self.setLayout(self.buttonLayout)        

    def exportData(self, png=False, csv=False, wav=False):
        if png:
            pngExporter = pyqtgraph.exporters.ImageExporter(self.exportArea.scene())
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Image", directory=self.defaultName+".png", filter="Images (*.png *.jpg)")
            if targetPath:
                img =  QImage(pngExporter.export(toBytes=True))
                img.save(targetPath)
        if csv:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Text", directory=self.defaultName+".csv", filter="Text (*.csv *.txt)")
            if targetPath:
                xs = np.array(self.xData)/self.sampleRate
                ys = np.array(self.yData)
                data = np.transpose((xs, ys))
                hdrString = "Data recorded and exported with Shiggytech's Infrasound. For More information visit https://github.com/Shiggymon/Infrasound"
                np.savetxt(fname=targetPath, X=data, fmt=("%g", "%.5g"), delimiter=";", header=hdrString)
        if wav:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Audio", directory=self.defaultName+".wav", filter="Audio (*.wav)")
            if targetPath:
                ys = np.array(self.yData)
                scalingFactor = np.iinfo(np.int16).max/np.max(ys)
                data = ys*scalingFactor
                wavfile.write(targetPath, round(self.sampleRate*128), data.astype(np.int16))
                
