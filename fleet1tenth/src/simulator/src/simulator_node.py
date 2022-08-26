#! /usr/bin/env python3

import numpy as np
import rospy
from scipy.integrate import odeint, solve_ivp


class SimulatedCar:
    def __init__(self, ID, model_data, initial_pose):
        self.ID=ID
        
        # physical params
        self.m=model_data["m"]
        self.l_r=model_data["l_r"]
        self.l_f=model_data["l_f"]
        self.I_z=model_data["I_z"]
        
        # drivetrain params
        self.C_m1=model_data["C_m1"]
        self.C_m2=model_data["C_m2"]
        self.C_m3=model_data["C_m3"]

        # tire params
        self.C_f=model_data["C_f"]
        self.C_r=model_data["C_r"]

        # init list for the states, use the initial_pose
        self.x=[initial_pose[0]]
        self.y=[initial_pose[1]]
        self.phi=[initial_pose[2]]
        # rest of the init values are 0
        self.v_xi=[0]
        self.v_eta=[0]
        self.omega=[0]

    def step(self, dt, method="RK45"):
        """
        Steps the simulation with the specified time step

        Arguments:
            - dt(float): Time step in [s]
            - method(str): Integration method to use. Available options:

                - "RK45"(default): Explicit Runge-Kutta method of order 5(4). The error is controlled assuming accuracy of the fourth-order method, but steps are taken using the fifth-order accurate formula (local extrapolation is done). 
                - "RK23": Explicit Runge-Kutta method of order 3(2). The error is controlled assuming accuracy of the second-order method, but steps are taken using the third-order accurate formula (local extrapolation is done)
                - "DOP853": Explicit Runge-Kutta method of order 8. Python implementation of the “DOP853” algorithm originally written in Fortran [14]. A 7-th order interpolation polynomial accurate to 7-th order is used for the dense output. Can be applied in the complex domain.
                - "Radau": Implicit Runge-Kutta method of the Radau IIA family of order 5. The error is controlled with a third-order accurate embedded formula.
                - "BDF": Implicit multi-step variable-order (1 to 5) method based on a backward differentiation formula for the derivative approximation.
                - "LSODA": Adams/BDF method with automatic stiffness detection and switching. This is a wrapper of the Fortran solver from ODEPACK.
        """
        states=solve_ivp(self.dynamics, [0, dt], self.get_current_states(), method=method)
        self.x.append(states[0])
        self.y.append(states[1])
        self.phi.append(states[2])
        self.v_xi.append(states[3])
        self.v_eta.append(states[4])
        self.omega.append(states[5])
        
        

    def get_current_states(self):
        """
        Returns the current value of the state variables
        """
        return np.array([self.x[-1],self.y[-1],self.phi[-1],self.v_xi[-1],self.v_eta[-1],self.omega[-1]])


    def dynamics(self, states, inputs):
        """
        Function that describes the model dynamics

        Arguments:
            -states(ndarray): 1D vector of the state variables 
        """
        # retrieve inputs & states
        d,delta=inputs
        x,y,phi,v_xi,v_eta,omega=states

        # calculate lognitudinal tire force
        F_xi=self.C_m1*d-self.C_m2*v_xi-np.sign(v_xi)*self.C_m3

        # obtain lateral tire forces
        try:
            alpha_f=-np.arctan((omega*self.l_f+v_eta)/v_xi)+delta
            alpha_r=np.arctan(omega*self.l_r-v_eta/v_xi)
            F_f_eta=self.C_f*alpha_f
            F_r_eta=self.C_r-alpha_r
        except ZeroDivisionError:
            F_f_eta=0
            F_r_eta=0

        # dynamics
        d_x=v_xi*np.cos(phi)-v_eta*np.sin(phi)
        d_y=v_xi*np.sin(phi)+v_eta*np.cos(phi)
        d_phi=omega

        d_v_xi=1/self.m*(F_xi+F_xi*np.cos(delta)-F_f_eta*np.sin(delta)+self.m*v_eta*omega)
        d_v_eta=1/self.m*(F_r_eta+F_xi*np.sin(delta)+F_f_eta*np.cos(delta)-self.m*v_xi*omega)
        d_omega=1/self.I_z*(F_f_eta*self.l_f*np.cos(delta)+F_xi*self.l_f*np.sin(delta)-F_r_eta*self.l_r)

        return np.array([d_x,d_y,d_phi,d_v_xi,d_v_eta,d_omega])