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
import time


class CombinedController(BaseController):
    def __init__(self, FREQUENCY, lateral_gains, longitudinal_gains, projection_window, projection_step, look_ahead=0):
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

        # look ahead distance for evaluation
        self.look_ahead=look_ahead

        # feedback gain polynomials
        self.k_lat1=np.poly1d(lateral_gains["k1"])
        self.k_lat2=np.poly1d(lateral_gains["k2"])
        self.k_lat3=np.poly1d(lateral_gains["k3"])

        self.k_long1=np.poly1d(longitudinal_gains["k1"])
        self.k_long2=np.poly1d(longitudinal_gains["k2"])


        self.logfile=open(expanduser("~")+"/car1.csv", "w")
        self.logfile.write(str(longitudinal_gains["k1"])+"\n")
        self.logfile.write("t,x,y,fi,vxi,veta,omega,d,delta,z,theta_e,s_err,v_err\n")



    def _state_callback(self, data):
        """
        Callback function triggered by the state subscriber that calculates the control inputs of the system.
        """
        
        ### CHECK IF THE CONTROLLER IS ENABLED
        if not self.enabled:
            return

        ### PROCESS STATE DATA ###    
        # get velocity data
        v_xi=data.velocity_x
        v_eta=data.velocity_y

        # get the current position
        position=np.array([ 
                            data.position_x+np.sign(v_xi)*self.look_ahead*np.cos(data.heading_angle),
                            data.position_y+np.sign(v_xi)*self.look_ahead*np.sin(data.heading_angle)
                            ]
                            ) # adaptive look ahead to reduce the effect of time delay
        phi=_normalize(data.heading_angle)

        # CoM side-slip
        beta=np.arctan2(v_eta,abs(v_xi)) # abs() to condiser backward motion ass well

        ### PROJECT ONTO PATH###
        # estimate path parameter velocity
        s_est=self.s+v_xi*self.dt
        

        # project
        self.s=project_to_closest(pos=position,s_est=s_est, s_window=self.projection_window,
                            path=self.get_path,step=self.projection_step, s_bounds=(self.s_start,self.s_end))


        # get path data at projected reference
        ref_pos, s0, z0, v_ref, c=self.get_path_data(self.s)

        # invert heading in case of backward motion
        if v_ref<0:
            phi+=np.pi

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
        delta=c*(0.0992*v_xi**2-0.0949)+theta_e/2-k_lat1*self.q-k_lat2*e-k_lat3*self.edot
                

        #longitudinal
        k_long1,k_long2=self.get_longitudinal_feedback_gains(p)
        d=-k_long1*(self.s-self.s_ref)-k_long2*(v_xi-v_ref)
        d+=(3.012*v_ref+0.6044*np.sign(v_xi))/61.3835 # TODO: currently hard coded feedforward.. consider using parameters instead
        

        ### PUBLISH INPUTS ###
        msg=InputValues()
        msg.d=d
        msg.delta=-delta

        self.pub.publish(msg)

        ### step reference parameter
        self.s_ref+=abs(self.get_path_speed(self.s_ref))*self.dt

        # self.logfile.write(f"{data.header.stamp.to_sec()},{position[0]},{position[1]},{phi},{v_xi},{v_eta},omega,,{delta},{z1},{theta_e}\n")
        self.logfile.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(rospy.Time.now().to_sec(),position[0],position[1],phi,v_xi,v_eta,data.omega,d,delta,z1,theta_e,self.s-self.s_ref,v_xi-v_ref)) # Python 2.7 compatible
        #data.header.stamp.to_sec() # -> stamp from optitrack 

    def get_lateral_feedback_gains(self, v_xi):
        k1=self.k_lat1(v_xi)
        k2=self.k_lat2(v_xi)
        k3=self.k_lat3(v_xi)
        return k1,k2,k3


    def get_longitudinal_feedback_gains(self,p):
        k1=self.k_long1(p)
        k2=self.k_long2(p)
        return k1, k2
