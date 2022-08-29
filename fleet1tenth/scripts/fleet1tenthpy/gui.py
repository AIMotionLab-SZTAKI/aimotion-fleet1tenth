#!/usr/bin/env python3

from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QLineEdit, QComboBox
from PyQt5.QtCore import pyqtSignal, Qt, QEvent, QTimer
from PyQt5.QtGui import QDoubleValidator
import sys


class GUIManager():
    def __init__(self):
        """
        Class Managing the PyQt GUI application
        """

        self.app=QApplication(sys.argv)

    def setKeyboardToControl(self):
        """
        Lowers the keyboard input interval for more accurate telemanipulation
        """
        self.app.setKeyboardInputInterval(10)


    def resetKeyboardToControl(self):
        """
        Resets the keyboard input interval 
        """
        self.app.setKeyboardInputInterval(400)



class VehicleChooserDialog(QDialog):
    def __init__(self, IDs):
        super(VehicleChooserDialog, self).__init__()
        self.setWindowTitle("fleet1tenth - Vehicle chooser")

        submitBTN=QPushButton("Choose")
        submitBTN.clicked.connect(self.accept)
        
        layout = QVBoxLayout()
        self.checkboxes=[]
        for ID in IDs:
            c = QCheckBox(ID)
            layout.addWidget(c)
            self.checkboxes.append(c)
        layout.addWidget(submitBTN)

        self.setLayout(layout)


    def getChecked(self):
        """
        Returns a list of strings with the chosen vehicle IDs.
        """
        return [
            checkbox.text() for checkbox in self.checkboxes if checkbox.isChecked()
        ]



class VehicleController(QDialog):
    # desine control signal to emit
    control=pyqtSignal(tuple) #(car_ID, d, delta) TODO check order to minimc the rest
    def __init__(self, car_IDs, FREQUENCY):
        """
        GUI app for the teleoperation of the vehicle
        """
        super(VehicleController, self).__init__()
        self.setWindowTitle("fleet1tenth - Vehicle Controller")

        self.installEventFilter(self)  # To register mouse clicks and key presses

        # ID for identification
        self.idCMB=QComboBox()
        self.idCMB.addItems(car_IDs)

        # init variables
        self.d_current=0
        self.delta_current=0
        self.d_max=0.075
        self.delta_max=0.5

        # Timer for calling the control signal emitter function periodically
        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.emitControl)
        self.timer.start(1000/FREQUENCY) # Start timer with the specified frequency

        
        # Duty cycle control
        dutyLBL=QLabel("Duty-cycle limit:")
        self.dutyLNE=QLineEdit()
        self.dutyLNE.setValidator(QDoubleValidator(0, 0.5, 3)) # validator maximum can be adjusted if needed
        self.dutyLNE.setText("0.075")
        self.dutyLNE.textChanged.connect(self.updateD)
        dutyHlayout=QHBoxLayout() # layout for better organization
        dutyHlayout.addWidget(dutyLBL)
        dutyHlayout.addWidget(self.dutyLNE)

        # steering angle control
        steeringLBL=QLabel("Steering angle limit:")
        self.steeringLNE=QLineEdit()
        self.steeringLNE.setValidator(QDoubleValidator(0, 0.6, 3)) # validator maximum can be adjusted if needed
        self.steeringLNE.setText("0.5")
        self.steeringLNE.textChanged.connect(self.updateDelta)
        steeringHlayout=QHBoxLayout() # layout for better organization
        steeringHlayout.addWidget(steeringLBL)
        steeringHlayout.addWidget(self.steeringLNE)

        
        closeBTN=QPushButton("Close")
        closeBTN.clicked.connect(self.accept)


        detailsLBL=QLabel()
        detailsLBL.setText("Usage:\n - KeyUP: forward\n - KeyDOWN: backward\n - KeyLeft: left\n - KeyRight: right\n - KeySHIFT: Escape from input field")
        detailsLBL.setWordWrap(True)
        
        
        layout = QVBoxLayout()
        layout.addWidget(self.idCMB)
        layout.addLayout(dutyHlayout)
        layout.addLayout(steeringHlayout)
        layout.addWidget(detailsLBL)
        layout.addWidget(closeBTN)

        self.setLayout(layout)

    def updateD(self):
        """
        Update duty cycle limit
        """
        try:
            self.d_max=float(self.dutyLNE.text())
        except ValueError:
            self.dutyLNE.setText(str(self.d_max))


        if self.d_current<-self.d_max:
            self.d_current=-self.d_max
        elif self.d_current>self.d_max:
            self.d_current=self.d_max


    def updateDelta(self):
        """
        Update steering angle limit
        """
        try:
            self.delta_max=float(self.steeringLNE.text())
        except ValueError:
            self.steeringLNE.setText(str(self.delta_max))

        if self.delta_current<-self.delta_max:
            self.delta_current=-self.delta_max
        elif self.delta_current>self.delta_max:
            self.delta_current=self.delta_max


    def eventFilter(self, source, event):
        """Catches keyboard events, modifies the control inputs"""

        # key press
        if event.type() == QEvent.KeyPress:
            key = event.key()
            if key==Qt.Key_Up:
                self.d_current=self.d_max
            elif key==Qt.Key_Down:
                self.d_current=-self.d_max
            elif key==Qt.Key_Left:
                self.delta_current=-self.delta_max
            elif key==Qt.Key_Right:
                self.delta_current=self.delta_max
            elif key==Qt.Key_Shift: # return focus instead of quitting
                self.setFocus()
                


        # key release
        if event.type() == QEvent.KeyRelease:
            key = event.key()
            if key==Qt.Key_Up and self.d_current>0:
                self.d_current=0
            elif key==Qt.Key_Down and self.d_current<0:
                self.d_current=0
            elif key==Qt.Key_Left and self.delta_current<0:
                self.delta_current=0
            elif key==Qt.Key_Right and self.delta_current>0:
                self.delta_current=0
            elif key==Qt.Key_Shift: # return focus instead of quitting
                self.setFocus()

        # call parent function
        return super(VehicleController, self).eventFilter(source, event)

    def accept(self):
        """
        Called before window close
        """
        self.timer.stop()
        super(VehicleController, self).accept()


    def reject(self):
        """
        Called before window close
        """
        self.timer.stop()
        super(VehicleController, self).reject()

    def emitControl(self):
        """
        Emits the control signals
        """
        self.control.emit((self.idCMB.currentText(), self.d_current, self.delta_current))



    def loop(self):
        """
        Creates an infinite loop while the controller is running
        """
        while self.timer.isActive():
            pass



class LoggerDialog(QDialog):
    start_logging=pyqtSignal(bool)
    stop_logging=pyqtSignal(bool)

    def __init__(self, car_ID):
        """
        GUI app for displaying logged vehicle state data
        """
        super(LoggerDialog, self).__init__()
        self.setWindowTitle("fleet1tenth - State logger")

        idLBL=QLabel(car_ID)

        # create labels + lineedits
        self.tLNE=QLineEdit()
        self.tLNE.setReadOnly(True)
        self.xLNE=QLineEdit()
        self.xLNE.setReadOnly(True)
        self.yLNE=QLineEdit()
        self.yLNE.setReadOnly(True)
        self.phiLNE=QLineEdit()
        self.phiLNE.setReadOnly(True)
        self.v_xiLNE=QLineEdit()
        self.v_xiLNE.setReadOnly(True)
        self.v_etaLNE=QLineEdit()
        self.v_etaLNE.setReadOnly(True)
        self.omegaLNE=QLineEdit()
        self.omegaLNE.setReadOnly(True)

        self.erpmLNE=QLineEdit()

        self.dLNE=QLineEdit()
        self.deltaLNE=QLineEdit()

        tLBL=QLabel("Time [s]")
        xLBL=QLabel("x-Position [m]")
        yLBL=QLabel("y-Position [m]")
        phiLBL=QLabel("Heading angle [rad]")
        v_xiLBL=QLabel("Longitudinal velocity [m/s]")
        v_etaLBL=QLabel("Lateral velocity [m/s]")
        omegaLBL=QLabel("Angular velocity [rad/s]")

        erpmLBL=QLabel("ERPM [rpm]")

        dLBL=QLabel("Motor reference [1]")
        deltaLBL=QLabel("Steering input [1]") # the input value sent to the steering servo not the steering angle in [rad]

        self.startBTN=QPushButton("Start Logging")
        self.startBTN.clicked.connect(self.emit_start_logging)
        self.stopBTN=QPushButton("Stop logging")
        self.stopBTN.clicked.connect(self.emit_stop_logging)

        # organize into layouts
        labelLayout=QVBoxLayout()
        labelLayout.addWidget(tLBL)
        labelLayout.addWidget(xLBL)
        labelLayout.addWidget(yLBL)
        labelLayout.addWidget(phiLBL)
        labelLayout.addWidget(v_xiLBL)
        labelLayout.addWidget(v_etaLBL)
        labelLayout.addWidget(omegaLBL)
        labelLayout.addWidget(erpmLBL)
        labelLayout.addWidget(dLBL)
        labelLayout.addWidget(deltaLBL)
        
        lineeditLayout=QVBoxLayout()
        lineeditLayout.addWidget(self.tLNE)
        lineeditLayout.addWidget(self.xLNE)
        lineeditLayout.addWidget(self.yLNE)
        lineeditLayout.addWidget(self.phiLNE)
        lineeditLayout.addWidget(self.v_xiLNE)
        lineeditLayout.addWidget(self.v_etaLNE)
        lineeditLayout.addWidget(self.omegaLNE)
        lineeditLayout.addWidget(self.erpmLNE)
        lineeditLayout.addWidget(self.dLNE)
        lineeditLayout.addWidget(self.deltaLNE)
        
        dataLayout=QHBoxLayout()
        dataLayout.addLayout(labelLayout)
        dataLayout.addLayout(lineeditLayout)

        buttonLayout=QHBoxLayout()
        buttonLayout.addWidget(self.startBTN)
        buttonLayout.addWidget(self.stopBTN)

        Layout=QVBoxLayout()
        Layout.addWidget(idLBL)
        Layout.addLayout(dataLayout)
        Layout.addLayout(buttonLayout)
        
        self.setLayout(Layout)
        self.show()

    def updateValues(self, time,x,y,heading_angle,ERPM,v_xi,v_eta,omega,d,delta):
        """
        Update the displayed state values
        """
        self.tLNE.setText(f"{time:.3f}")
        self.xLNE.setText(f"{x:.3f}")
        self.yLNE.setText(f"{y:.3f}")
        self.phiLNE.setText(f"{heading_angle:.3f}")
        self.erpmLNE.setText(f"{ERPM:.1f}")
        self.v_xiLNE.setText(f"{v_xi:.3f}")
        self.v_etaLNE.setText(f"{v_eta:.3f}")
        self.omegaLNE.setText(f"{omega:.3f}")
        self.dLNE.setText(f"{d:.3f}")
        self.deltaLNE.setText(f"{delta:.3f}")


    def emit_start_logging(self):
        self.start_logging.emit(True)
        self.startBTN.setEnabled(False)
        self.stopBTN.setEnabled(True)

    def emit_stop_logging(self):
        self.stop_logging.emit(True)
        self.startBTN.setEnabled(True)
        self.stopBTN.setEnabled(False)


    


    