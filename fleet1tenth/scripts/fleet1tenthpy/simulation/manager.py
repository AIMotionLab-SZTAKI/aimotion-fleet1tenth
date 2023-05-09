import rospy
from vehicle_state_msgs.msg import VehicleStateStamped
import numpy as np
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import Rectangle
import matplotlib
from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QComboBox,QFileDialog
import pandas as pd
matplotlib.use("Qt5Agg")


class PlaybackDialog(QDialog):
    def __init__(self, car_IDs):
        """
        Class responsible for the management of the simulation

        Arguments:
            - car_IDs(str,list): Unique indentifier of the simulated vehicles
        """
        super(PlaybackDialog, self).__init__()
        self.car_IDs=car_IDs
        self.car_logs=[]

        for ID in self.car_IDs:
            l=CarLogs(ID)
            self.car_logs.append(l)


    def init_ui(self):
        self.setWindowTitle("fleet1tenth - Vehicle visualization")

        #playback visualization layout
        #self.carCHB=QCheckBox("Show vehicles")
        #self.pathCHB=QCheckBox("Show paths")

        #playbacksettingsLayout=QHBoxLayout()
        #playbacksettingsLayout.addWidget(self.carCHB)
        #playbacksettingsLayout.addWidget(self.pathCHB)

        #playbacksettingsGB=QGroupBox()
        #playbacksettingsGB.setTitle("Playback settings")
        #playbacksettingsGB.setLayout(playbacksettingsLayout)
        
        self.vidFIG=FigAnimation(self.car_logs)
        #self.vidLBL = VideoLabel()
        #self.vidLBL.setAlignment(Qt.AlignCenter)
        #self.vidLBL.setMinimumSize(400, 400)
        #self.vidLBL.setStyleSheet("background-color: #ffffff")


        self.exportVidBTN=QPushButton("Export video")
        self.exportVidBTN.clicked.connect(self.export_video)

        btnLayout=QHBoxLayout()
        btnLayout.addWidget(self.exportVidBTN)

        vidplaybackLayout=QVBoxLayout()
        #vidplaybackLayout.addWidget(playbacksettingsGB)
        vidplaybackLayout.addWidget(self.vidFIG)
        vidplaybackLayout.addLayout(btnLayout)


        self.carCMB=QComboBox()
        self.carCMB.addItems([car.ID for car in self.car_logs])
        exportcardataBTN=QPushButton("Export logged data")
        exportcardataBTN.clicked.connect(self.export_data)

        exportLayout=QHBoxLayout()
        exportLayout.addWidget(self.carCMB)
        exportLayout.addWidget(exportcardataBTN)


        #widget layout
        layout=QVBoxLayout()
        layout.addLayout(vidplaybackLayout)
        layout.addLayout(exportLayout)
        self.setLayout(layout)

        

    def export_data(self):
        logs=next((x for x in self.car_logs if x.ID == self.carCMB.currentText()), None)
        data=np.zeros((len(logs.t), 9))
        data[:,0]=np.asarray(logs.t)
        data[:,1]=np.asarray(logs.x)
        data[:,2]=np.asarray(logs.y)
        data[:,3]=np.asarray(logs.phi)
        data[:,4]=np.asarray(logs.v_xi)
        data[:,5]=np.asarray(logs.v_eta)
        data[:,6]=np.asarray(logs.omega)
        data[:,7]=np.asarray(logs.d)
        data[:,8]=np.asarray(logs.delta)
        df = pd.DataFrame(data, columns=["t [s]","x [m]","y [m]","phi [rad]","v_xi [m/s]","v_eta [m/s]","omega [rad/s]", "d [1]", "delta [rad]"])
        save_name = QFileDialog.getSaveFileName(
                self,
                "Save data",
                "/",
                "CSV file (*.csv);;Text file (*.txt);;Excel file (*.xlsx)",
            )
        if save_name[0] != "":
            #if save_name[1] == "Excel file (*.xlsx)":
            #    df.to_excel(save_name[0])
            #else:
            df.to_csv(save_name[0])


    def export_video(self):
        save_name = QFileDialog.getSaveFileName(
                self,
                "Save video",
                "/",
                "GIF file (*.gif)",
            )
        if save_name[0] != "":
            #if save_name[1] == "Excel file (*.xlsx)":
            #    df.to_excel(save_name[0])
            #else:
            self.vidFIG.save(save_name[0]+".gif", writer="ffmpeg")

    def open_dialog(self):
        self.init_ui()
        self.exec_()

    def close(self) -> bool:
        self.vidFIG.close_event()
        return super().close()
        



class FigAnimation(FigureCanvas, TimedAnimation):
    def __init__(self, logs):
        #load data
        self.logs=logs
        self.t=self.logs[0].t

        # create figure
        self.fig=Figure(figsize=(5,5),dpi=150)
        self.ax=self.fig.add_subplot(111)

        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")

        self.ax.set_xlim(min(min(l.x for l in self.logs))-0.1,max(max(l.x for l in self.logs))+0.1)
        self.ax.set_ylim(min(min(l.y for l in self.logs))-0.1,max(max(l.y for l in self.logs))+0.1)
        self.ax.grid()
        self.ax.set_aspect("equal")

        self.lines=[]
        self.rects=[]

        for i in range(len(logs)):
            l=Line2D([], [])
            x=logs[i].x[0]
            y=logs[i].y[0]
            phi=logs[i].phi[0]
            xr=x-0.165*np.cos(phi)-0.1*np.cos(np.pi/2+phi)
            yr=y-0.165*np.sin(phi)-0.1*np.sin(np.pi/2+phi)
            rect=Rectangle(xy=(xr,yr), width=0.33, height=0.2, angle=np.rad2deg(phi))
            self.lines.append(l)
            self.ax.add_line(l)
            self.ax.add_patch(rect)
            

        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self,self.fig,interval=50, blit=True)


    def _draw_frame(self, framedata):
        # delete rectangles
        [p.remove() for p in reversed(self.ax.patches)]

        for i in range(len(self.lines)):
            #self.ax.plot(self.logs[i].x[0:framedata], self.logs[i].y[0:framedata])
            self.lines[i].set_data(self.logs[i].x[0:framedata], self.logs[i].y[0:framedata])

            x=self.logs[i].x[framedata]
            y=self.logs[i].y[framedata]
            phi=self.logs[i].phi[framedata]
            xr=x-0.165*np.cos(phi)-0.1*np.cos(np.pi/2+phi)
            yr=y-0.165*np.sin(phi)-0.1*np.sin(np.pi/2+phi)
            rect=Rectangle(xy=(xr,yr), width=0.33, height=0.2, angle=np.rad2deg(phi), color="black", zorder=10)
            self.ax.add_patch(rect)
    
            


    def new_frame_seq(self):
        return iter(range(len(self.t)))

    def _init_draw(self):
        for l in self.lines:
            l.set_data([], [])




class CarLogs:
    def __init__(self, ID):
        """
        Class to store logged vehicle data

        Arguments:
            - ID(str): Unique identifier of the logged vehicle
        """
        
        self.ID=ID

        # empty lists for the incoming data
        self.t=[]

        self.x=[]
        self.y=[]
        self.phi=[]

        self.v_xi=[]
        self.v_eta=[]
        self.omega=[]

        self.d=[]
        self.delta=[]

        self.sub=rospy.Subscriber("/"+self.ID+"/state", VehicleStateStamped, callback=self._log_callback)


    def _log_callback(self, data):
        time=float(str(data.header.stamp.secs)+str(data.header.stamp.nsecs).zfill(9))

        self.t.append(time)

        self.x.append(data.position_x)
        self.y.append(data.position_y)
        self.phi.append(data.heading_angle)
        self.v_xi.append(data.velocity_x)
        self.v_eta.append(data.velocity_y)
        self.omega.append(data.omega)
        self.d.append(data.duty_cycle)
        self.delta.append(data.delta)

    def get_position_data(self):
        return self.x, self.y, self.phi

    def get_velocity_data(self):
        return self.v_xi, self.v_eta, self.omega

    def get_input_data(self):
        return self.d, self.delta

    def get_time(self):
        return self.t
