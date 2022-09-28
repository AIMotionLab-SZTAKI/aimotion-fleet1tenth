#!/usr/bin/env python

import rospy
#from service_definitions.srv import TrajectorySrv, TrajectorySrvResponse
from scipy.interpolate import splev
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import math
import numpy as np

class BaseController:
    def __init__(self,FREQUENCY):
        """
        Base class for the implementation of path following controllers
        """
        # freq
        self.dt=1.0/FREQUENCY

        # setup vehicle state reciever
        self.state_subscriber=rospy.Subscriber("state", VehicleStateStamped, self._state_callback)
         
        # controller enable flag
        self.enabled=False # if True enable state callbacks that trigger the control

        self.pub=rospy.Publisher("control", InputValues,queue_size=1)

        
    def spin(self):
        """Prevents the loop from exiting"""
        rospy.spin()


    def _execute_trajectory(self, trajectory_data):
        """
        Callback function that recieves trajectories from the command PC
        """
        # set reference trajectory
        t=trajectory_data.path_t
        c=trajectory_data.path_c
        k=trajectory_data.path_k
        self.trajectory_tck=(t,c,k)

        # set reference speed profile
        t=trajectory_data.v_t
        c=trajectory_data.v_c
        k=trajectory_data.v_k
        self.speed_tck=(t,c,k)

        #set trajectory lenght
        self.s=trajectory_data.s_start
        self.s_start=trajectory_data.s_start
        self.s_end=trajectory_data.s_end
        self.s_ref=self.s_start

        # enable state callbacks that trigger the control
        self.enabled=True
        print("Executing trajectory...")
        while self.enabled:
            pass # wait for the controller to run

        #if self.success:
        #    return TrajectorySrvResponse(True)
        #return TrajectorySrvResponse(False)


    def _state_callback(self, data):
        # compute control inputs & publish-> inplemented in subclass
        pass

    def get_path(self,s):
        """
        Returns the path position at parameter s

        Arguments:
            - s(float): Path parameter/arc length
        """

        (x, y) = splev(s, self.trajectory_tck)
        return np.array([x,y])


    def get_path_data(self, s):
        """
        Helper function that returns path information at the specified parameter

        Arguments:
            - s(float): Parameter(arc length) of the Spline representing the path
        """
        # position & derivatives
        (x, y) = splev(s, self.trajectory_tck)
        (x_, y_) = splev(s, self.trajectory_tck, der=1)
        (x__,y__)=splev(s, self.trajectory_tck,der=2)
        
        # calculate base vectors of the moving coordinate frame
        s0 = np.array(
            [x_ / np.sqrt(x_**2 + y_**2), y_ / np.sqrt(x_**2 + y_**2)]
        )
        z0 = np.array(
            [-y_ / np.sqrt(x_**2 + y_**2), x_ / np.sqrt(x_**2 + y_**2)]
        )

        # calculate path curvature
        c=abs(x__*y_-x_*y__)/((x_**2+y_**2)**(3/2))

        # get speed reference
        #v = splev(s, self.speed_tck)
        v=3#TODO: speed profile from spline!

        return np.array([x, y]), s0, z0, v, c

    def shutdown(self):
        if self.enabled:
            self.enabled=False


def _clamp(value, bound):
    """
    Helper function that clamps the given value with the specified bounds

    Arguments:
        - value(float): The value to clamp
        - bound(tuple/int/float): If int/float the function constrains the value into [-bound,bound]
                                  If tuple the value is constained into the range of [bound[0],bound[1]]
    """
    if isinstance(bound, int) or isinstance(bound, float):
        if value < -bound:
            return -bound
        elif value > bound:
            return bound
        return value
    elif isinstance(bound, tuple):
        if value < bound[0]:
            return bound[0]
        elif value > bound[1]:
            return bound[1]
        return value

def project_to_closest(pos, s_est, s_window, path, step, s_bounds):
    """
    Projects the current vehicle position onto the closest position of the path

    Arguments:
        - pos(ndarray): x,y position of the vehicle
        - s_est(float): the estimated s arc length parameter of the path
        - window_s(float): The lenth of thew projection window
        - path(float): Function that returns the x,y position of the path at s
        - step(float): Step size between the points of evaluation in the projection window
        - s_bounds(tuple): The lower (s_bound[0]) and upper bound (s_bound[1]) of s
    """

    floor = _clamp(s_est - s_window / 2, s_bounds)
    ceil = _clamp(s_est + s_window, s_bounds)
    window = np.linspace(floor, ceil, round((ceil - floor) / step))
    path_points = path(window).T

    deltas = path_points - pos
    indx = np.argmin(np.einsum("ij,ij->i", deltas, deltas))
    return floor + indx * step


def _normalize(angle):
    """
    Normalizes the given angle into the [-pi/2, pi/2] range

    Arguments:
        - angle(float): The angle to normalize, in radian
    """
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi

    return angle