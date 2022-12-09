#!/usr/bin/env python3

from os import environ
import yaml
import rospy
from drive_bridge_msg.msg import InputValues
import roslaunch
from roslaunch.core import RLException
from pathlib import Path
import time
from ..gui import GUIManager, VehicleChooserDialog, VehicleController
from ..logging import StateLogger
from ..install_utils import create_clients, create_environment
import actionlib
from control.msg import trajectoryAction, trajectoryGoal
from .manager import PlaybackDialog
import atexit
# import pygame


class Car:

    def __init__(self,ID, PARAMS_DICT):
        """
        Class interface for the communication and control of one SIMULATED F1/10 vehicle
        
        Arguments:
            - ID(str): Unique string used to identify the vehicle. Must be the same as the name of
                  the correspopndig RigidBody defined in the OptiTrack system.

                            
        """
        
        # for identification
        self.ID=ID

        
        # control params
        self.LATERAL_CONTROL_GAINS=PARAMS_DICT["control"]["LATERAL_CONTROL_GAINS"]
        self.LONGITUDINAL_CONTROL_GAINS=PARAMS_DICT["control"]["LONGITUDINAL_CONTROL_GAINS"]


        self.running=False # might be unused (TODO)
        self.launch=None

        # create ROS control publisher
        self.control_pub=rospy.Publisher(ID+"/control", InputValues, queue_size=1)

        # setup ROS action clients
        self.execute_trajectory_client=actionlib.SimpleActionClient(ID+"/execute_trajectory", trajectoryAction)
        

    def install_system(self, **kwargs):
        print("This function is not implemented in simulation mode!")


    def launch_system(self):
        """
        Launches communication, control and driver nodes onboard the F1/10 vehicle
        
        """
        # set ROS parameters
        rospy.set_param("/"+self.ID+"/path_following_control_node/lateral_gains", self.LATERAL_CONTROL_GAINS)
        rospy.set_param("/"+self.ID+"/path_following_control_node/longitudinal_gains", self.LONGITUDINAL_CONTROL_GAINS)

        # Configure logging
        uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # setup CLI arguments TODO: modify for simulation
        cli_args=[str(Path(__file__).parents[3])+'/src/start/launch/simulated_vehicle.launch',f'car_id:={self.ID}']
        roslaunch_args=cli_args[1:]
        roslaunch_file=[(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        # Launch
        try:
            self.launch=roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.launch.start()
        except RLException:
            raise Exception()


        print(f"\n###############################\nLaunched {self.ID}...\n###############################\n")
        time.sleep(2) # wait for nodes to launch
        self.running=True


    def check_steering(self):
        """
        Function that provides a visual feedback that the system started successfully and th connection is established
        """
        pass # not implemented in simulation


    def control(self, d, delta):
        """
        Sends direct control commands to the vehicle

        Arguments:
            - d(float): PWM duty cycle input of the motor. Valid range is between -1 and 1.
            - delta (float): Steering angle input. Valid range is between -0.6 and 0.6 but the actuator node might clip it,
                             if mechanical boundaries are reached. 
        """

        # create message
        msg=InputValues()
        msg.d=d
        msg.delta=delta
        
        # publish
        self.control_pub.publish(msg)

    def shutdown(self):
        """
        Shuts down the nodes running onboard the vehicle.
        """

        self.running=False
        self.launch.shutdown()
        print(f"\n###############################\n{self.ID} has shut down!\n###############################\n")



    def init_logger(self, path, gui=False):
        """
        Initializes the state logger for the vehicle
        """
        self.logger=StateLogger(self.ID, path, gui)


    def start_logging(self):
        """
        Starts logging the states to the file specified in the init_logger() function
        """
        pass
        try:
            self.logger.start_logging()
        except AttributeError:
            print("State logger must be initialized before calling the start_logging/stop_logging functions!")
    


    def stop_logging(self):
        """Stops the state logger"""
        try:
            self.logger.stop_logging()
        except:
            print("State logger must be initialized before calling the start_logging/stop_logging functions!")


    def execute_trajectory(self, path_tck, speed_tck, range):
        if not self.execute_trajectory_client.wait_for_server(timeout=rospy.Duration(secs=2)):
            raise Exception("Unable to upload the trajectory: Control action server timed out!")
        
        goal=trajectoryGoal()
        
        # 2D path data
        goal.path_t=path_tck[0].tolist()
        goal.path_cx=path_tck[1][0].tolist()
        goal.path_cy=path_tck[1][1].tolist()
        goal.path_k=path_tck[2]

        # 1D speed profile data
        goal.speed_t=speed_tck[0].tolist()
        goal.speed_c=speed_tck[1].tolist()
        goal.speed_k=speed_tck[2]

        goal.s_start=range[0]
        goal.s_end=range[1]

        self.execute_trajectory_client.send_goal(goal, feedback_cb=None) # TODO: consider adding feedback function
        print(f"{self.ID} exectuing the uploaded trajectory...")

        self.execute_trajectory_client.wait_for_result()
        
        result=self.execute_trajectory_client.get_result()

        if result:
            print(f"{self.ID} executed the uploaded trajectory successfully!")
        else:
            print(f"{self.ID} failed to execute the uploaded trajectory!")
        return result



class Fleet:
    def __init__(self, config_file):
        """
        Class responsible for handling a fleet of F1/10 cars
        
        Arguments:
            - config_file(str):  Path string to the yaml file containing all the
                            configuration data needed for the setup
        
        """
        if config_file is not None:
            config_file=config_file # in simulation needed for the launch of the simulator node
        else:
            config_file=str(Path(__file__).parents[2])+"/config/simulation.yaml"

        self.simulation_launch=None

        try:
            with open(config_file, "r") as f:
                try:
                    self.config_data=yaml.safe_load(f)
                except yaml.YAMLError as e:
                    print("Cannot load the YAML configuration file!")
                    return
        except IOError:
            print(f"No configuration file found at the specified path: {config_file}")
            raise RuntimeError(f"No configuration file found at the specified path: {config_file}")


        # ROS environment variables
        environ["ROS_MASTER_URI"]=self.config_data["ROS_MASTER_URI"]
        environ["ROS_IP"]=self.config_data["ROS_IP"] 

        # System frequency for the controllers, state observers...
        self.FREQUENCY=self.config_data["FREQUENCY"]
        rospy.set_param("/AIMotionLab/FREQUENCY", self.FREQUENCY)



        # Load vehicle config data
        self.vehicle_data=self.config_data["vehicles"]

        # init list to store Car objects
        self.cars=[]

        # init GUI manager
        self.GUIManager=GUIManager()


        self.playback_manager=None

        # initialize ROS node
        try:
            # check if master is running
            rospy.get_master().getSystemState()
            rospy.init_node("FleetMASTER")
        except:
            raise Exception(f"Unable to contact ROS master at {environ['ROS_MASTER_URI']}")


    def choose_cars(self):
        """
        Launches a simple GUI application to choose which cars to initialize a control interface for,
        from the fleet defined in the configuration file
        """
        dialog=VehicleChooserDialog(IDs=[car_ID for car_ID in self.vehicle_data])
        if dialog.exec_():
            car_list=dialog.getChecked()
            dialog.deleteLater()
            return car_list
        else:
            dialog.deleteLater()
            return []

    
    def init_cars(self, IDs=None):
        """
        Launches a chooser application to select vehicle's to use the initializes the control interface for them
        """
        self.drop_cars()
        if IDs is not None:
            if isinstance(IDs, str):
                c=Car(IDs, PARAMS_DICT=self.vehicle_data[IDs])
                print(f"Vehicle control interface initialized for {c.ID}")
                self.cars.append(c)
            elif isinstance(IDs, list) or isinstance(IDs, tuple):
                for id in IDs:
                    c=Car(id, self.vehicle_data[id],PARAMS_DICT=self.vehicle_data[id])
                    print(f"Vehicle control interface initialized for {c.ID}")
                    self.cars.append(c)
            else:
                raise Exception("Unsupported vatiable type for IDs! Accepted: str,list,tuple")

        else:
            chosen=self.choose_cars()
            for id in chosen:
                c=Car(id,  PARAMS_DICT=self.vehicle_data[id])
                print(f"Vehicle control interface initialized for {c.ID}")
                self.cars.append(c)


    def drop_cars(self):
        """
        Drops the current initialized control interfaces
        """
        for car in self.cars:
            if car.running:
                car.shutdown()
            print(f"Deleting {car.ID} control interface...")
            del car
        self.cars=[]

    def launch_simulator(self):
        """Launches the simulator node of the system"""
        # launch the simulator node if its not already running
        # Configure logging
        uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # setup CLI arguments
        cli_args=[str(Path(__file__).parents[3])+'/src/start/launch/simulator.launch']
        roslaunch_args=cli_args[1:]
        roslaunch_file=[(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        # set ROS params for the simulator
        rospy.set_param("/SIMULATOR/simulator_node/SOLVER", self.config_data["SOLVER"])
        rospy.set_param("/SIMULATOR/simulator_node/default_vehicle_model", self.config_data["default_vehicle_model"])
        rospy.set_param("/SIMULATOR/simulator_node/vehicles", self.config_data["vehicles"])
        rospy.set_param("/SIMULATOR/simulator_node/launched_vehicles", [c.ID for c in self.cars])


        # Launch
        try:
            self.simulation_launch=roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.simulation_launch.start()
        except RLException:
            raise Exception()

        self.playback_manager=PlaybackDialog([c.ID for c in self.cars])
        atexit.register(self.playback_manager.open_dialog)

        

    def launch_cars(self, IDs=None):
        """
        Launches the control and actuator nodes on the specified vehicles
        
        Arguments:
            - IDs(list/str): ID list to specitfy which to start
                   If not provided connect to all cars specified in config_file
        """

        # create a list of failed interfaces to delete later
        failed=[]

        if IDs is None: # start all cars
            for car in self.cars:
                try:
                    car.launch_system()
                    #car.check_steering()
                except Exception as e:
                    print(f"Unable establish connection to {car.ID}, deleting interface...")
                    failed.append(car)
    
        else: # start only the specified cars only
            for car in self.cars:
                if car.ID in IDs:
                    try:
                        car.launch_system()
                        # car.check_steering() unnecessary for simulation
                    except Exception as e:
                        print(e)
                        print(f"Unable establish connection to {car.ID}, deleting interface...")
                        failed.append(car)
        
        # if any of the launchers fail delete the control interface
        for fail in failed:
            self.cars.remove(fail)
            del fail

        #### SIMULATION ONLY ####

        if self.simulation_launch is None:
            self.launch_simulator()

        time.sleep(2) # sleep to enable node startup before processing further


    def open_playback(self):
        self.playback_manager.show_gui()


    def shutdown_cars(self, ID=None):
        """
        Shuts down the nodes running on the specified cars

        Arguments:
            - ID(list/str): The ID of one car (str) of a list of car IDs (list) to shut down
                  If None it shut downs all running cars.
        """
        if ID is None:
            for car in self.cars:
                if car.running:
                    car.shutdown()
        else:
            for car in self.cars:
                if car.ID in ID and car.running:
                    car.shutdown()


    def install_vehicle_system(self):
        print("The install_vehicle_system function is not implemented in simulation mode!")


    def keyboard_remote(self):
        print("The keyboard remote is not implemented in simulation mode because of timing issues!")


    def control_car(self, inputs):
        """
        Sends control inputs to one specified car


        Arguments:
            - inputs(tuple): (car_ID, d, delta) where car_ID is the car to control
                              and d and delta are the control inputs
        """

        # get interface
        car=self.get_car_by_ID(inputs[0])

        # apply control
        car.control(inputs[1], inputs[2])


    def get_car_by_ID(self, ID):
        """
        Helper function that return the control interface for the car specified by ID

        Arguments:
            - ID: the ID of the selected control interface
        """

        car=next((c for c in self.cars if c.ID == ID), None)
        if car is None:
            raise Exception(f"No control interface for {ID}")
        return car