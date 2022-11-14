import matplotlib.pyplot
from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QLineEdit, QComboBox, QGroupBox
from PyQt5.QtGui import QMouseEvent,QWheelEvent
from PyQt5.QtCore import pyqtSignal
#import opencv2 as cv2

class Visualization(QWidget):
    def __init__(self, IDs):
        super(Visualization, self).__init__()
        self.setWindowTitle("fleet1tenth - Vehicle visualization")
        self.IDs=IDs

        #playback visualization layout
        self.carCHB=QCheckBox("Show vehicles")
        self.pathCHB=QCheckBox("Show paths")

        playbacksettingsLayout=QVBoxLayout()
        playbacksettingsLayout.addWidget(self.carCHB)
        playbacksettingsLayout.addWidget(self.pathCHB)

        playbacksettingsGB=QGroupBox()
        playbacksettingsGB.setTitle("Playback settings")
        playbacksettingsGB.setLayout(playbacksettingsLayout)


        

        #widget layout
        layout=QHBoxLayout()
        self.setLayout(layout)


class VideoLabel(QLabel):
    """Label to display the visualization frames"""

    # create signals
    press = pyqtSignal(float, float)
    moving = pyqtSignal(float, float)
    release = pyqtSignal(float, float)
    wheel = pyqtSignal(float)

    def __init__(self, parent=None):
        """Intitialization"""
        self.press_pos = None
        self.current_pos = None
        super(VideoLabel, self).__init__(parent)

    def wheelEvent(self, a0: QWheelEvent):
        """Handles wheel event, emits signal with zoom parameter"""
        if a0.angleDelta().y() > 0:
            self.wheel.emit(-0.1)
        else:
            self.wheel.emit(0.1)
        return super().wheelEvent(a0)

    def mousePressEvent(self, ev: QMouseEvent):
        """Handles mouse press event, emits coordinates"""
        x_label, y_label, = ev.x(), ev.y()

        if self.pixmap():
            label_size = self.size()
            pixmap_size = self.pixmap().size()
            width = pixmap_size.width()
            height = pixmap_size.height()

            x_0 = int((label_size.width() - width) / 2)
            y_0 = int((label_size.height() - height) / 2)

            if (
                x_label >= x_0
                and x_label < (x_0 + width)
                and y_label >= y_0
                and y_label < (y_0 + height)
            ):
                x_rel = (x_label - x_0 - width / 2) / width
                y_rel = (y_label - y_0 - height / 2) / height
                self.press_pos = (x_rel, y_rel)
                self.press.emit(x_rel, y_rel)
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev: QMouseEvent):
        """Handles mouse movement, emits coordinates"""
        x_label, y_label, = ev.x(), ev.y()

        if self.pixmap():
            label_size = self.size()
            pixmap_size = self.pixmap().size()
            width = pixmap_size.width()
            height = pixmap_size.height()

            x_0 = int((label_size.width() - width) / 2)
            y_0 = int((label_size.height() - height) / 2)

            if (
                x_label >= x_0
                and x_label < (x_0 + width)
                and y_label >= y_0
                and y_label < (y_0 + height)
            ):
                x_rel = (x_label - x_0 - width / 2) / width
                y_rel = (y_label - y_0 - height / 2) / height
                self.moving.emit(x_rel, y_rel)
                self.current_pos = (x_rel, y_rel)
        super().mousePressEvent(ev)

    def mouseReleaseEvent(self, ev: QMouseEvent):
        """Handles mouse release, emits coordinates"""
        x_label, y_label, = ev.x(), ev.y()

        if self.pixmap():
            label_size = self.size()
            pixmap_size = self.pixmap().size()
            width = pixmap_size.width()
            height = pixmap_size.height()

            x_0 = int((label_size.width() - width) / 2)
            y_0 = int((label_size.height() - height) / 2)

            if (
                x_label >= x_0
                and x_label < (x_0 + width)
                and y_label >= y_0
                and y_label < (y_0 + height)
            ):
                x_rel = (x_label - x_0 - width / 2) / width
                y_rel = (y_label - y_0 - height / 2) / height
                self.release.emit(x_rel, y_rel)
        super().mousePressEvent(ev)