import matplotlib.pyplot
from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QLineEdit, QComboBox

class SimulatorVisualization:
    def __init__(self, IDs):
        super(SimulatorVisualization, self).__init__()
        self.setWindowTitle("fleet1tenth - Vehicle simulator visualization")

        layout=QVBoxLayout()
        self.setLayout(layout)

    