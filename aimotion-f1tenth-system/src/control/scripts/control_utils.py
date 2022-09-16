from urllib import response
import rospy
from service_definitions.srv import TrajectorySrv, TrajectorySrvResponse
from scipy.interpolate import splev
from vehicle_state_msgs.msg import VehicleStateStamped
import math

class BaseController:
    def __init__(self, *args, **kwargs):

        # setup vehicle state reciever
        self.VehicleState=None # stores vehicle state data
        self.state_subscriber=rospy.Subscriber("/state", VehicleStateStamped, self._state_callback)
        
        # trajectory service
        self.trajectory_service=rospy.Service("execute_trajectory", TrajectorySrv, self._execute_trajectory)
         
        # controller enable flag
        self.enabled=False # if True enable state callbacks that trigger the control

        self.pub=rospy.Publisher("/control")

        
    def spin():
        """Prevents the loop from exiting"""
        rospy.spin()


    def _execute_trajectory(self, trajectory_data):
        """
        Callback function that recieves trajectories from the command PC
        """
        # set reference trajectory
        t=trajectory_data.t
        c=trajectory_data.c
        k=trajectory_data.k
        self.trajectory_tck=(t,c,k)

        self.enabled=True
        while self.enabled:
            pass # wait for the controller to run

        if self.success:
            return TrajectorySrvResponse(True)
        return TrajectorySrvResponse(False)


    def _state_callback(self, data):
        if not self.running:
            return # only execute callbacks if the controller is enabled

        self.VehicleState=data

        # compute control inputs & publish-> inplemented in subclass


def 