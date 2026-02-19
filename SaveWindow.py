from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QPushButton, QFileDialog
from PyQt6.QtGui import QImage
from datetime import datetime
from pyqtgraph import GraphicsLayoutWidget
import pyqtgraph.exporters
import numpy as np
from scipy.io import wavfile
import tempfile, zipfile, os

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
        buttonExportAll = QPushButton("Save All (ZIP)")
        self.buttonLayout.addWidget(buttonExportPng)
        self.buttonLayout.addWidget(buttonExportCsv)
        self.buttonLayout.addWidget(buttonExportWav)
        self.buttonLayout.addWidget(buttonExportAll)
        buttonExportPng.clicked.connect(lambda :self.exportData(png=True))
        buttonExportCsv.clicked.connect(lambda :self.exportData(csv=True))
        buttonExportWav.clicked.connect(lambda :self.exportData(wav=True))
        buttonExportAll.clicked.connect(lambda :self.exportData(all=True))
        
        self.setLayout(self.buttonLayout)
        for b in (buttonExportPng, buttonExportCsv, buttonExportWav):
            b.setDefault(False)
            b.setAutoDefault(False)

    def exportData(self, png=False, csv=False, wav=False, all=False):
        targetPath = ""
        if png:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Image", directory=self.defaultName+".png", filter="Images (*.png *.jpg)")
            if targetPath:
                self.savePng(targetPath)
        if csv:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Text", directory=self.defaultName+".csv", filter="Text (*.csv *.txt)")
            if targetPath:
                self.saveCsv(targetPath)
        if wav:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save Audio", directory=self.defaultName+".wav", filter="Audio (*.wav)")
            if targetPath:
                self.saveWav(targetPath)
        if all:
            targetPath, _ = QFileDialog.getSaveFileName(caption="Save All", directory=self.defaultName+".zip", filter="ZIP Archive (*.zip)")
            if targetPath:
                with tempfile.TemporaryDirectory(prefix="shiggytech.infra") as tmpDir:
                    pngPath = os.path.join(tmpDir, self.defaultName + ".png")
                    self.savePng(pngPath)
                    csvPath = os.path.join(tmpDir, self.defaultName + ".csv")
                    self.saveCsv(csvPath)
                    wavPath = os.path.join(tmpDir, self.defaultName + ".wav")
                    self.saveWav(wavPath)
                    with zipfile.ZipFile(targetPath, "w", zipfile.ZIP_DEFLATED) as z:
                        z.write(pngPath, os.path.basename(pngPath))
                        z.write(csvPath, os.path.basename(csvPath))
                        z.write(wavPath, os.path.basename(wavPath))
        if targetPath:
            self.accept()
    
    def savePng(self, targetPath):
        pngExporter = pyqtgraph.exporters.ImageExporter(self.exportArea.scene())
        img =  QImage(pngExporter.export(toBytes=True))
        img.save(targetPath)
    
    def saveCsv(self, targetPath):
        xs = np.array(self.xData)/self.sampleRate
        ys = np.array(self.yData)
        data = np.transpose((xs, ys))
        hdrString = "Data recorded and exported with Shiggytech's Infrasound. For More information visit https://github.com/Shiggymon/Infrasound"
        np.savetxt(fname=targetPath, X=data, fmt=("%g", "%.5g"), delimiter=";", header=hdrString)
    
    def saveWav(self, targetPath):
        ys = np.array(self.yData)
        scalingFactor = np.iinfo(np.int16).max/np.max(ys)
        data = ys*scalingFactor
        wavfile.write(targetPath, round(self.sampleRate*128), data.astype(np.int16))