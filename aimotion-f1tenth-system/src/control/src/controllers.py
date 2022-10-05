#!/usr/bin/env python

# Developed in Python 2.7

import numpy as np
from control_utils import BaseController, project_to_closest, _normalize, _clamp
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import math
import rospy
import time
from os.path import expanduser

class CombinedController(BaseController):
    def __init__(self, FREQUENCY, projection_window, projection_step):
        """
        Implementation of the path tracking control algorithm:
                -> Lateral control: Upgraded Stanley method using lateral error dynamics
                -> Longitudinal control: State feedback control based on the longitudinal dynamics
        """
        super(CombinedController, self).__init__(FREQUENCY=FREQUENCY)
        
        # required params for feedback control
        self.projection_window=projection_window
        self.projection_step=projection_step
        
        # Lateral dynamics integral term
        self.q=0

        self.logfile=open(expanduser("~")+"/car1.csv", "w")
        self.logfile.write(f"t,x,y,fi,vxi,veta,omega,d,delta,z,theta_e\n")



    def _state_callback(self, data):
        """
        Callback function triggered by the state subscriber that calculates the control inputs of the system.
        """
        
        ### CHECK IF THE CONTROLLER IS ENABLED
        if not self.enabled:
            return

        ### PROCESS STATE DATA ###
        # get the current position
        position=np.array([data.position_x, data.position_y])
        phi=data.heading_angle

        # get velocity data
        v_xi=data.velocity_x
        v_eta=data.velocity_y

        # CoM side-slip
        beta=np.arctan2(v_eta,v_xi)

        ### PROJECT ONTO PATH###
        # estimate path parameter velocity
        s_est=self.s+v_xi*self.dt
        

        # project
        self.s=project_to_closest(pos=position,s_est=s_est, s_window=self.projection_window,
                            path=self.get_path,step=self.projection_step, s_bounds=(self.s_start,self.s_end))


        # get path data at projected reference
        ref_pos, s0, z0, v_ref, c=self.get_path_data(self.s)

        ### CHECK CURRENT POSITION & PROVIDE ROS ACTION FEEDBACK ###
        if self.check_progress():
            return # if the goal is reached exit the function


        ### MODEL STATES IN PATH COORDINATES
        # path tangent angle 
        theta_p = np.arctan2(s0[1], s0[0])

        # lateral error
        z1=np.dot(position-ref_pos, z0)

        # heading error
        theta_e=_normalize(phi-theta_p)

        # longitudinal model parameter
        p=abs(np.cos(theta_e+beta)/np.cos(beta)/(1-c*z1))


        # invert z1 for lateral dynamics:
        e=-z1
        self.q+=e

        # estimate error derivative
        try:
            self.edot=0.5*((e-self.ep)/self.dt-self.edot)+self.edot
            self.ep=e
        except AttributeError:
            self.edot=0
            self.ep=e

        ### FEEDBACK CONTROL ###
        # lateral
        k_lat1,k_lat2,k_lat3=self.get_lateral_feedback_gains(v_xi)
        delta=theta_e-k_lat1*self.q-k_lat2*e-k_lat3*self.edot

        #longitudinal
        k_long1,k_long2=self.get_longitudinal_feedback_gains(p)
        d=-k_long1*(self.s-self.s_ref)-k_long2*(v_xi-v_ref)

        

        ### PUBLISH INPUTS ###
        msg=InputValues()
        msg.d=d
        msg.delta=-delta

        self.pub.publish(msg)

        ### step reference parameter
        self.s_ref+=v_ref*self.dt

        self.logfile.write(f"t,{position[0]},{position[1]},{phi},{v_xi},{v_eta},omega,,{delta},{z1},{theta_e}\n")

    def get_lateral_feedback_gains(self, v_xi):
        if v_xi>0:
            k1=-0.0387*v_xi**3+0.2249*v_xi**2-0.4541*v_xi+0.5137
            k2=-0.8997*v_xi**3+5.2284*v_xi**2-10.5470*v_xi+11.8980
            k3=+0.1246*v_xi**3-0.7735*v_xi**2+1.7601*v_xi+0.6148
            return k1,k2,k3
        else:
            return 0,0,0


    def get_longitudinal_feedback_gains(self,p):
        k1=-0.0185*p+0.5
        k2=+0.0118*p+0.5
        return k1, k2









        
        
        