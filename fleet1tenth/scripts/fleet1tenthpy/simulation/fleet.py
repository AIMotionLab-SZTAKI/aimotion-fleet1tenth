#!/usr/bin/env python3

from os import environ
import yaml
import rospy
from drive_bridge_msg.msg import InputValues
import roslaunch
from roslaunch.core import RLException
from pathlib import Path
import time
from .gui import GUIManager, VehicleChooserDialog, VehicleController
from .logging import StateLogger
from .install_utils import create_clients, create_environment
# import pygame


class Car:

    def __init__(self,ID, *args):
        """
        Class interface for the communication and control of one SIMULATED F1/10 vehicle
        
        Arguments:
            - ID(str): Unique string used to identify the vehicle. Must be the same as the name of
                  the correspopndig RigidBody defined in the OptiTrack system.

                            
        """
        
        # for identification
        self.ID=ID


        self.running=False # might be unused (TODO)
        self.launch=None

        # create ROS control publisher
        self.control_pub=rospy.Publisher(ID+"/control", InputValues, queue_size=1)
        

    def install_system(self, **kwargs):
        print("This function is not implemented in simulation mode!")


    def launch_system(self, **kwargs):
        """
        Launches communication, control and driver nodes onboard the F1/10 vehicle
        
        Arguments:
            - OPTITRACK_SERVER_IP(str): Local IP adress of the motion capture server from which the car
                                   recieves it's position data
            - FREQUENCY(flat): The system update frequency used by the state estimator and control nodes
        """

        # Configure logging
        uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # setup CLI arguments TODO: modify for simulation
        cli_args=[str(Path(__file__).parents[2])+'/src/start/launch/simulated_vehicle.launch', f'machine_ip:={self.IP_ADRESS}',f'username:={self.USERNAME}',f'password:={self.PASSWORD}',f'car_id:={self.ID}', f'optitrack_ip:={OPTITRACK_SERVER_IP}', f'tracker_offset:={self.MARKER_OFFSET}',f'update_frequency:={FREQUENCY}',f'angle_gain:={self.STEERING_GAINS[0]}', f'angle_offset:={self.STEERING_GAINS[1]}']
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






class Fleet:
    def __init__(self, config_file):
        """
        Class responsible for handling a fleet of F1/10 cars
        
        Arguments:
            - config_file(str):  Path string to the yaml file containing all the
                            configuration data needed for the setup
        
        """
        self.config_file=config_file # in simulation needed for the launch of the simulator node

        self.launch=None

        try:
            with open(config_file, "r") as f:
                try:
                    config_data=yaml.safe_load(f)
                except yaml.YAMLError as e:
                    print("Cannot load the YAML configuration file!")
                    return
        except IOError:
            print(f"No configuration file found at the specified path: {config_file}")
            raise RuntimeError(f"No configuration file found at the specified path: {config_file}")


        # ROS environment variables
        environ["ROS_MASTER_URI"]=config_data["ROS_MASTER_URI"]
        environ["ROS_IP"]=config_data["ROS_IP"] 

        # System frequency for the controllers, state observers...
        self.FREQUENCY=config_data["FREQUENCY"]

        # Load vehicle config data
        self.vehicle_data=config_data["vehicles"]

        # init list to store Car objects
        self.cars=[]

        # init GUI manager
        self.GUIManager=GUIManager()

        # initialize ROS node
        try:
            # check if master is running
            rospy.get_master().getSystemState()
            rospy.init_node("FleetMASTER")
        except:
            raise Exception(f"Unable to contact ros master at {environ['ROS_MASTER_URI']}")


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

    
    def init_cars(self):
        """
        Launches a chooser application to select vehicle's to use the initializes the control interface for them
        """
        self.drop_cars()
        chosen=self.choose_cars()
        for id in chosen:
            c=Car(id, self.vehicle_data[id])
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
                    car.launch_system(self.FREQUENCY)
                    car.check_steering()
                except Exception:
                    print(f"Unable establish connection to {car.ID}, deleting interface...")
                    failed.append(car)
    
        else: # start only the specified cars only
            for car in self.cars:
                if car.ID in IDs:
                    try:
                        car.launch_system(self.OPTITRACK_SERVER_IP, self.FREQUENCY)
                        # car.check_steering() unnecessary for simulation
                    except Exception:
                        print(f"Unable establish connection to {car.ID}, deleting interface...")
                        failed.append(car)
        
        # if any of the launchers fail delete the control interface
        for fail in failed:
            self.cars.remove(fail)
            del fail

        #### SIMULATION ONLY ####
        # launch the simulator node if its not already running
        if self.launch is not None:
            # Configure logging
            uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            # setup CLI arguments
            cli_args=[str(Path(__file__).parents[2])+'/src/start/launch/simulator.launch', f'simulation_yaml_path={self.config_file}', f'launched_vehicles={IDs}']
            roslaunch_args=cli_args[1:]
            roslaunch_file=[(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

            # Launch
            try:
                self.launch=roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.launch.start()
            except RLException:
                raise Exception()


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