#!/usr/bin/env python3

import rospy
from vehicle_state_msgs.msg import VehicleStateStamped
from os.path import expanduser
from time import gmtime, strftime
from .gui import LoggerDialog

class StateLogger:
    def __init__(self, car_ID, path, gui=False):
        """
        Class responsible for logging the full-state of the specified car.

        Arguments:
            - car_ID(str): Unique identifier of the vehicle
            - path(str): Path of the logfile
            - gui(bool): If set to True the app opens a new window to display the actual data
        """

        self.car_ID=car_ID
        self.path=path

        self.sub=None
        self.file=None
        self.i=0

        # store the ROS timestamp of the first logged frame for conversion
        self.t0=None

        # init gui if required
        if gui:
            self.init_GUI()
        else:
            self.GUI=None


    def open_file(self,path):
        """
        Opens a log file on the specified path

        Arguments:
            - path(str): Log file path
        """

        filename=strftime(expanduser("~")+"/"+path+"_%Y-%m-%d-%H-%M-%S", gmtime())+".csv"
        print(f"Log file opened: {filename}")
        self.file=open(filename, "w")
        self.file.write("t [s],x-Position [m],y-Position [m],Heading angle [rad],ERPM [rpm],Longitudinal velocity [m/s],Lateral velocity[m/s],Angular velocity [rad/s],d [1], Steering input [1]\n")


    def init_GUI(self):
        """
        Initializes the state logger GUI
        """

        self.GUI=LoggerDialog(self.car_ID)
        self.GUI.rejected.connect(self.stop_logging)
        self.GUI.start_logging.connect(self.start_logging)
        self.GUI.stop_logging.connect(self.stop_logging)


    def log_data(self, data):
        """
        Callback function for the state subscriber

        Arguments:
            - data(VehicleStateStamped): Vehicle state message
        """

        # retrieve ROS timestamp
        time=float(str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs).zfill(9))
        
        # save the timestamp of the first logged frame
        if self.t0 is None:
            self.t0=time
        
        # convert from ROS time
        time=time-self.t0

        # log to file
        self.file.write(f"{time},{data.position_x},{data.position_y},{data.heading_angle},{data.ERPM},{data.velocity_x},{data.velocity_y},{data.omega},{data.duty_cycle},{data.delta}\n")

        # update gui if necessary
        if self.GUI is not None:
            self.i=self.i+1
            if self.i>10:
                self.GUI.updateValues(time,data.position_x,data.position_y,
                                        data.heading_angle,data.ERPM,data.velocity_x,
                                        data.velocity_y,data.omega,data.duty_cycle,data.delta)
                self.i=0


    def stop_logging(self):
        """
        Unsubscribes from the vehicle's state topic and closes the file
        """

        if self.sub is not None:
            self.sub.unregister()
            self.sub=None

        if self.file is not None:
            self.file.close()
            self.file=None
            print(f"\n###############################\nState logging stopped for {self.car_ID}...\n###############################\n")

        

    def start_logging(self):
        """
        Subscribes to state topic starts the logging
        """

        self.t0=None
        self.open_file(self.path)
        self.sub=rospy.Subscriber("/"+self.car_ID+"/state",VehicleStateStamped, self.log_data)

        print(f"\n###############################\nStarted state logger for {self.car_ID}...\n###############################\n")