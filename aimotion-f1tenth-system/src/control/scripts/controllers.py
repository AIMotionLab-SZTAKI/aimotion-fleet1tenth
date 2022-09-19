#!/usr/bin/env python

# Developed in Python 2.7

import numpy as np
from .control_utils import BaseController, project_to_closest, _normalize, _clamp
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues

class FeedBackController(BaseController):
    def __init__(self, FREQUENCY, projection_window, projection_step):
        super(FeedBackController, self).__init__(FREQUENCY=FREQUENCY)
        
        # required params for feedback control
        self.projection_window=projection_window
        self.projection_step=projection_step
        
        # reference position for state feedback
        self.s_ref=self.s_start



    def _state_callback(self, data):
        super()._state_callback(data)

        # get the current position
        position=np.array([data.position_x, data.position_y])

        # estimate path parameter velocity
        s_est=self.VehicleState.velocity_x*self.dt

        s=project_to_closest(pos=position,s_est=s_est, s_window=self.projection_window,
                            path=self.get_path,step=self.projection_step, s_bounds=(self.s_start,self.s_end))


        # get path data at projected reference
        ref_pos, s0, z0, v_ref, c=self.get_path_data(s)

        # check for success
        if s-self.s_end<0.05:
            self.enabled=False
            self.success=True

        ### LONGITUDINAL CONTROL ###
        
        # CoM side-slip
        beta=np.arctan(data.velocity_y/data.velocity_x)

        # Path tangent angle at s
        theta_p = np.arctan2(s0[1], s0[0])

        # lateral error
        z1=np.dot(position-ref_pos, z0)

        # calculate scheduled feedback
        p=np.cos(data.heading_angle+beta-theta_p)/np.cos(beta)/(1-c*z1)
        k_long1,k_long2=self.get_longitudinal_feedback_gains(p)

        # calculate control input
        d=-k_long1*(s-self.s_ref)-k_long2*(data.velocity_x-v_ref)

        ### LATERAL CONTROL ###
        
        # lateral error == z1 (alrready known)
        
        #heading error:
        theta_e=_normalize(data.heading_angle-theta_p)

        # estimate derivatives
        try:
            z1_dot=(z1-self.z1_prev)/self.dt
            theta_e_dot=(theta_e-self.theta_e_prev)/self.dt
            self.z1_prev=z1
            self.theta_e_prev=theta_e

        except AttributeError:
            # at the first timestep the derivatives cannot be estimated
            # just save the data for the next iteration
            z1_dot=0
            theta_e_dot=0
            self.z1_prev=z1
            self.theta_e_prev=theta_e

        # get feedback gains
        k_lat1,k_lat2,k_lat3,k_lat4=self.get_lateral_feedback_gains(data.velocity_y)
        
        # calculate control input
        delta=-k_lat1*z1-k_lat2*z1_dot-k_lat3*theta_e-k_lat4*theta_e_dot

        # step path rerefence 
        self.s_ref=data.veloctiy_x*self.dt

        msg=InputValues()
        msg.d=d
        msg.delta=delta


        
    def get_lateral_feedback_gains(self, v_xi):
        return 1,1,1,1


    def get_longitudinal_feedback_gains(self,p):
        return 1,1









        
        
        