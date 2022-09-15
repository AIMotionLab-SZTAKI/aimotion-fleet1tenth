from urllib import response
import rospy
from control.srv import TrajectorySrv, TrajectorySrvResponse
from scipy.interpolate import splev
from vehicle_state_msgs.msg import VehicleStateStamped
import math

class Controller:
    def __init__(self):

        # setup vehicle state reciever
        self.VehicleState=None # stores vehicle state data
        self.state_subscriber=rospy.Subscriber("state", VehicleStateStamped, self._state_callback)
        
        # trajectory service
        self.trajectory_service=rospy.Service("upload_trajectory", TrajectorySrv, self._upload_trajectory)

    def spin():
        """Prevents the loop from exiting"""
        rospy.spin()


    def _upload_trajectory(self, trajectory_data):
        """
        Callback function that recieves trajectories from the command PC
        """
        if self.validate_trajectory(trajectory_data):
            return TrajectorySrvResponse(True)
        return TrajectorySrvResponse(False)

    def validate_trajectory(self, trajectory_data):
        """
        Checks if the uploaded trajectory originates from the current vehicle position
        """
        path_tck=trajectory_data.path.tck
        p0=splev(0,path_tck)
        if math.dist(p0,self.get_position)<0.1 and True: # heading angle difference
            return True
        return False


    def _state_callback(self, data):
        self.VehicleState=data