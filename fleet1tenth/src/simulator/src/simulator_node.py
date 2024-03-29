#! /usr/bin/env python3

import numpy as np
import rospy
import time
from drive_bridge_msg.msg import InputValues
from vehicle_state_msgs.msg import VehicleStateStamped
from math import ceil

def RungeKutta4_step(t, h, y_n, model, inputs):
    """Executes 1 step of the Runge-Kutta method with input params kept as a constant"""
    k1 = model(t, y_n, inputs)
    k2 = model(t + h / 2, y_n + h * k1 / 2, inputs)
    k3 = model(t + h / 2, y_n + h * k2 / 2, inputs)
    k4 = model(t + h, y_n + h * k3, inputs)
    y_n1 = y_n + 1 / 6 * h * (k1 + 2 * k2 + 2 * k3 + k4)
    t_n1 = t + h
    return t_n1, y_n1

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

def _clamp(value, bound):
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


class SimulatedCar:
    def __init__(self, ID, model_data, default_model, FREQUCENCY):
        """
        Class responsible for the simulation of the vehicle dynamics for the specified car

        Arguments:
            - ID(str): Unique identifier of the simulated vehicle.
            - model_data(dict): Parameters of the simualted dynamic single track model
            - initial_pose(list): [x,y,phi] Initial position and orientation of the vehicle
        """


        # identification
        self.ID=ID

        # publisher & subscribers
        self.pub=None
        self.sub=None
        
        #input values
        self.d=[0]
        self.delta=[0]

        self.dt=1.0/FREQUCENCY

        if self.dt>0.001:
            self.num_of_steps=ceil(self.dt/0.001)
            self.stepsize=self.dt/self.num_of_steps
        else:
            self.stepsize=None

        # physical params
        try: # try custom params
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

            self.delta_max=model_data["delta_max"]
            self.d_max=model_data["d_max"]
        
        except KeyError: # in case of key error return to default model
            self.m=default_model["m"]
            self.l_r=default_model["l_r"]
            self.l_f=default_model["l_f"]
            self.I_z=default_model["I_z"]

            # drivetrain params
            self.C_m1=default_model["C_m1"]
            self.C_m2=default_model["C_m2"]
            self.C_m3=default_model["C_m3"]

            # tire params
            self.C_f=default_model["C_f"]
            self.C_r=default_model["C_r"]

            self.delta_max=default_model["delta_max"]
            self.d_max=default_model["d_max"]


        # init list for the states, use the initial_pose
        self.x=[model_data["init_pose"][0]]
        self.y=[model_data["init_pose"][1]]
        self.phi=[model_data["init_pose"][2]]

        # rest of the init values are 0
        self.v_xi=[0]
        self.v_eta=[0]
        self.omega=[0]
        self.t=[0]


    def step(self, inputs,t):
        """
        Steps the simulation with the specified time step

        Arguments:
            - inputs(tuple): The condtol inputs of the system (d, delta)
            - t(float): Simulation time
        """

        inputs=list(inputs)
        inputs[0]=_clamp(inputs[0],self.d_max)
        
        inputs[1]=_clamp(inputs[1],self.delta_max)
        
        self.d.append(inputs[0])
        self.delta.append(inputs[1])

        #sol=solve_ivp(fun=self.dynamics, t_span=(0, dt), y0=self.get_current_states(),t_eval=np.linspace(0, dt, 10), method=method)
        #states=sol.y
        #self.x.append(states[0][9])
        #self.y.append(states[1][9])
        #self.phi.append(_normalize(states[2][9]))
        #self.v_xi.append(states[3][9])
        #self.v_eta.append(states[4][9])
        #self.omega.append(states[5][9])
        
        states=self.get_current_states()
        if self.stepsize is not None:
            for _ in range(self.num_of_steps):
                _, states=RungeKutta4_step(0,self.stepsize,states,self.dynamics, inputs)        
        else:
            _, states=RungeKutta4_step(0,self.dt,states,self.dynamics, inputs)

        #states=RungeKutta4_step(0,self.dt,states,self.dynamics, inputs)


        self.x.append(states[0])
        self.y.append(states[1])
        self.phi.append(_normalize(states[2]))
        self.v_xi.append(states[3])
        self.v_eta.append(states[4])
        self.omega.append(states[5])

        self.t.append(t)
        #time.sleep(1)
        
        
        
        
    def get_current_states(self):
        """
        Returns the current value of the state variables
        """
        return np.array([self.x[-1],self.y[-1],self.phi[-1],self.v_xi[-1],self.v_eta[-1],self.omega[-1]])


    def dynamics(self, t, states, inputs):
        """
        Function that describes the model dynamics

        Arguments:
            -states(ndarray): 1D vector of the state variables 
        """
        # retrieve inputs & states
        d, delta = inputs
        
        # retrieve inputs & states
        x,y,phi,v_xi,v_eta,omega=states

        # calculate lognitudinal tire force
        F_xi=self.C_m1*d-self.C_m2*v_xi-np.sign(v_xi)*self.C_m3

        # obtain lateral tire forces
        if abs(v_xi)>0.1:# only use tire model for moving vehicles
            alpha_f=-(omega*self.l_f+v_eta)/(v_xi)+delta
            alpha_r=(omega*self.l_r-v_eta)/(v_xi)
                
            F_f_eta=self.C_f*alpha_f
            F_r_eta=self.C_r*alpha_r
            
        else:
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

    def publish_state(self):
        """
        Publishes the current state of the vehicle to the relevant topic
        """

        # generate message
        msg=VehicleStateStamped()
        msg.header.stamp=rospy.Time.from_sec(self.t[-1])
        msg.position_x=self.x[-1]
        msg.position_y=self.y[-1]
        msg.heading_angle=self.phi[-1]
        msg.velocity_x=self.v_xi[-1]
        msg.velocity_y=self.v_eta[-1]
        msg.delta=self.delta[-1]
        msg.duty_cycle=self.d[-1]
        msg.ERPM=0

        # publish
        self.pub.publish(msg)
    
   



class Simulator:
    def __init__(self, FREQUENCY, SOLVER, vehicle_data, default_model, launched_vehicles):
        self.FREQUENCY=FREQUENCY
        self.SOLVER=SOLVER
        self.vehicle_data=vehicle_data

        self.started=False # remains False until the simulator recieves control input
        self.timer=0
        self.cb_timer=0

        self.cars=[]
        self.subs=[]
        self.pubs=[]

        self.input_matrix={} # the input matrix is responsible for storing the current control inputs of each vehicle
        # it makes sure that the simulation is in sync for all the vehicles
        # the simulation only steps if all the cars are provided an input by the controller
        # the structure iof the matrix : input_matrix[car_ID]=[valid(bool), (d(float), delta(float))]
        
        # launch all the cars if no ID provided
        if launched_vehicles == "None":
            for car_ID in vehicle_data:
                c=SimulatedCar(car_ID, vehicle_data[car_ID], default_model, FREQUENCY)
                self.cars.append(c)
        
        # only launch the defined cars
        else:
            for car_ID in vehicle_data:
                if car_ID in launched_vehicles:
                    c=SimulatedCar(car_ID, vehicle_data[car_ID],default_model, FREQUENCY)
                    self.cars.append(c)

        # init the subscribers and publishers of the vehicle and create input matrix
        for c in self.cars:
            c.sub=rospy.Subscriber("/"+c.ID+"/control", InputValues, callback=self._input_callbacks, callback_args=c.ID)
            c.pub=rospy.Publisher("/"+c.ID+"/state",VehicleStateStamped, queue_size=1)
            self.input_matrix[c.ID]=[False, (0,0)]


        # create input matrix

        print(f"Simulation environment initialized for: {[c.ID for c in self.cars]}")
    

    def spin(self):
        """
        Function that publishes the initial
        """
        
        # after the simulation has started spin to prevent exiting
        while not rospy.is_shutdown():
            time.sleep(0.1)
            self.cb_timer+=1
            if self.cb_timer>=10:
                for c in self.cars:
                    c.publish_state()
                self.cb_timer=0



    
    def _input_callbacks(self,data, ID):
        """Handles the control inputs sent from the controller"""
        if not self.started:
            self.started=True # disable initial pose publishing

        # check validity: if already valid step the simulation for all valid data
        if self.input_matrix[ID][0]:
            # already valid the other vehicle controllers did not start yet
            # step the simulation for the valid inputs
            for key, value in self.input_matrix.items():
                c=next((c for c in self.cars if c.ID == key), None)
                self.timer+=1.0/self.FREQUENCY
                c.step(self.input_matrix[c.ID][1], self.timer)
                

            # publish if all the simulation were stepped to reduce delay
            for key, value in self.input_matrix.items():
                c=next((c for c in self.cars if c.ID == key), None)
                c.publish_state()
            self.reset_input_matrix()

            # store current input in matrix
            self.input_matrix[ID]=[True, (data.d, data.delta)]


        else: # normal operation
            # modify matrix
            self.input_matrix[ID]=[True, (data.d, data.delta)]
        
            if self.validate_input_matrix():
                for c in self.cars:
                    self.timer+=1.0/self.FREQUENCY
                    c.step(self.input_matrix[c.ID][1],self.timer)
                    
            
                for c in self.cars:# another for loop to reduce delay between the published messages
                    c.publish_state()
                self.reset_input_matrix()
            
        self.cb_timer=0



    def reset_input_matrix(self):
        """
        Resets the input matrix
        """
        for key in self.input_matrix:
            self.input_matrix[key]=[False, (0,0)]



    def validate_input_matrix(self):
        """Checks is inputs were recieved for all the vehicles"""
        for key, value in self.input_matrix.items():
            if not value[0]:
                return False
        return True


    def shutdown(self):
        self.started=True # to exit loop

# execute node
if __name__=="__main__":
    try:
        rospy.init_node("aimotion_simulator")
        simulator=Simulator(FREQUENCY=rospy.get_param("~FREQUENCY", default=20.0), 
                            SOLVER=rospy.get_param("~SOLVER", default="RK45"),
                            vehicle_data=rospy.get_param("~vehicles"),
                            default_model=rospy.get_param("~default_vehicle_model"),
                            launched_vehicles=rospy.get_param("~launched_vehicles", "None"))
        rospy.on_shutdown(simulator.shutdown)
        simulator.spin()
    except rospy.ROSInterruptException:
        pass
