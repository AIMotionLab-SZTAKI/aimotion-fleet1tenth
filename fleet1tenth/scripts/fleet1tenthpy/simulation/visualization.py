import matplotlib.pyplot
from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QLineEdit, QComboBox

class Visualization(QWidget):
    def __init__(self, IDs):
        super(Visualization, self).__init__()
        self.setWindowTitle("fleet1tenth - Vehicle visualization")
        self.IDs=IDs
        layout=QVBoxLayout()
        self.setLayout(layout)

    