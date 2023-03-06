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
from control.msg import trajectoryAction, trajectoryGoal
import actionlib


class Car:
    def __init__(self,ID, PARAMS_DICT):
        """
        Class interface for the communication and control of one F1/10 vehicle
        
        Arguments:
            - ID(str): Unique string used to identify the vehicle. Must be the same as the name of
                  the correspopndig RigidBody defined in the OptiTrack system.
            - PARAMS_DICT(dict): Contains the necessay params for initialization.
                            IP_ADRESS(str): IP adress of the vehicle in the local Wifi network
                            USERNAME(str): SSH conection username
                            PASSWORD(str): SSH connection password
                            STEERING_GAINS(list): Radian to control input transformation (u_s=delta*STEERING_GAINS[1]+STEERING_GAINS[0])
                            MARKER OFFSET(float): Distance of the defined RigidBody position from the vehicle's CoM in [m]
        """
        
        # for identification
        self.ID=ID

        # SSH client
        # self.client=SSHConnection(PARAMS_DICT["IP_ADRESS"],PARAMS_DICT["USERNAME"],PARAMS_DICT["PASSWORD"])
        # Connection parameters
        self.IP_ADRESS=PARAMS_DICT["IP_ADRESS"]
        self.USERNAME=PARAMS_DICT["USERNAME"]
        self.PASSWORD=PARAMS_DICT["PASSWORD"]

        # TODO: consider adding modelling info instead of storing data onboard
        #self.STEERING_GAINS=PARAMS_DICT["STEERING_GAINS"]
        #self.MARKER_OFFSET=PARAMS_DICT["MARKER_OFFSET"]
        #self.MOTOR_LIMIT=PARAMS_DICT["MOTOR_LIMIT"]

        #self.LATERAL_CONTROL_GAINS=PARAMS_DICT["control"]["LATERAL_CONTROL_GAINS"]
        #self.LONGITUDINAL_CONTROL_GAINS=PARAMS_DICT["control"]["LONGITUDINAL_CONTROL_GAINS"]

        self.MOCAP_SOURCE=PARAMS_DICT["MOCAP_SOURCE"]
        #try:
        #    self.MOCAP_EXTERNAL_TOPIC=PARAMS_DICT["MOCAP_EXTERNAL_TOPIC"]
        #except KeyError:
        #    if self.MOTION_CAPTURE=="external":
        #        raise Exception("MOTION_CAPTURE is set to 'external', but no topic name is provided. To resolve this issue set MOCAP_EXTERNAL_TOPIC is the configuration file.")

        self.running=False
        self.launch=None

        # create ROS control publisher
        self.control_pub=rospy.Publisher(ID+"/control", InputValues, queue_size=1)
        
        # setup ROS action clients
        self.execute_trajectory_client=actionlib.SimpleActionClient(ID+"/execute_trajectory", trajectoryAction)
        

    def update_param(self, param_name, value):
        """
        Funtion for updating the control & state estimator parameters from code, without modifying the default values in the configuration file. 
        To complete the update procedure this function also restarts the onboard software stack.

        Arguments:
            - param_name(str): Must be one of the following list: [STEERING_GAINS,MARKER_OFFSET,LATERAL_CONTROL_GAINS,LONGITUDINAL_CONTROL_GAINS,MOTOR_LIMIT]
            - value(float/list/dict): The value used to update the specified parameter. Type must match the one used in the configuration file. TODO: check if datatypes match raise exceptions
        """
        if param_name=="STEERING_GAINS":
            self.STEERING_GAINS=value
        elif param_name=="MARKER_OFFSET":
            self.MARKER_OFFSET=value
        elif param_name=="LATERAL_CONTROL_GAINS":
            self.LATERAL_CONTROL_GAINS=value
        elif param_name=="LONGITUDINAL_CONTROL_GAINS":
            self.LONGITUDINAL_CONTROL_GAINS=value
        elif param_name=="MOTOR_LIMIT":
            self.MOTOR_LIMIT=value
        else:
            print("Unkown parameter name!")

        self.shutdown()
        self.launch_system()




    def install_system(self, ROS_MASTER_URI):
        """
        Installs the aimotion-f1tenth-system software stack into the vehicle.

        Arguments:
            - ROS_MASTER_URI: IP adress of the PC running roscore
        """
        print(f"Installing aimotion-f1tenth-system on {self.ID}")

        # create connections
        SSH_client, SFTP_client=create_clients(self.IP_ADRESS, self.USERNAME, self.PASSWORD)

        # create env.sh file
        create_environment(ROS_MASTER_URI, self.IP_ADRESS, str(Path(__file__).parents[3])+"/aimotion-f1tenth-system/env.sh")

        
        # copy files
        print("Copying workspace onto vehicle...")
        SFTP_client.rmall("aimotion-f1tenth-system")
        SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)
        SFTP_client.put_dir(str(Path(__file__).parents[3])+"/aimotion-f1tenth-system", "aimotion-f1tenth-system")
        SFTP_client.close()

        # make .py & .sh files executable
        SSH_client.exec_command('find ~/aimotion-f1tenth-system/ -type f -name "*.py" -exec chmod u+x {} \;')
        SSH_client.exec_command('find ~/aimotion-f1tenth-system/ -type f -name "*.sh" -exec chmod u+x {} \;')
        
        # build ROS workspace
        print("Building ROS workspace...")
        stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/melodic/setup.bash; cd aimotion-f1tenth-system; catkin_make"')
        if stdout.channel.recv_exit_status():
            print("Failed to build ROS workspace")
            return

        print(f"Successfully installed aimotion-f1tenth-system on vehicle {self.ID}")


    def launch_system(self):
        """
        Launches communication, control and driver nodes onboard the F1/10 vehicle
        """

        # set ROS parameters
        #rospy.set_param("/"+self.ID+"/drive_bridge/angle_gain", self.STEERING_GAINS[0])
        #rospy.set_param("/"+self.ID+"/drive_bridge/angle_offset", self.STEERING_GAINS[1])
        #rospy.set_param("/"+self.ID+"/drive_bridge/reference_limit", self.MOTOR_LIMIT)

        #rospy.set_param("/"+self.ID+"/state_observer_node/tracker_offset", self.MARKER_OFFSET)
        rospy.set_param("/"+self.ID+"/state_observer_node/MOCAP_SOURCE", self.MOCAP_SOURCE)
        #if self.MOTION_CAPTURE == "external":
        #    try:
        #        rospy.set_param("/"+self.ID+"/state_observer_node/mocap_external_topic", self.MOCAP_EXTERNAL_TOPIC)
        #    except KeyError:
        #        raise Exception("MOTION_CAPTURE is set to external but no external topic name is provided! To resolve this issue specify MOCAP_EXTERNAL_TOPIC in the configuration file")

        #rospy.set_param("/"+self.ID+"/path_following_control_node/lateral_gains", self.LATERAL_CONTROL_GAINS)
        #rospy.set_param("/"+self.ID+"/path_following_control_node/longitudinal_gains", self.LONGITUDINAL_CONTROL_GAINS)

        # Configure logging
        uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # setup CLI arguments
        cli_args=[str(Path(__file__).parents[2])+'/src/start/launch/vehicle.launch', f'machine_ip:={self.IP_ADRESS}',f'username:={self.USERNAME}',f'password:={self.PASSWORD}',f'car_id:={self.ID}', f'motion_capture:={self.MOCAP_SOURCE}']
        roslaunch_args=cli_args[1:]
        roslaunch_file=[(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        # Launch
        try:
            self.launch=roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.launch.start()
        except RLException as e:
            print(e)
            raise Exception()


        print(f"\n###############################\nLaunched {self.ID}...\n###############################\n")
        time.sleep(2) # wait for nodes to launch
        self.running=True


    def check_steering(self):
        """
        Function that provides a visual feedback that the system started successfully and th connection is established
        """
        self.control(d=0, delta=0.4)
        time.sleep(0.3)
        self.control(d=0, delta=-0.4)
        time.sleep(0.1)
        self.control(d=0, delta=0)
        print(f"\n###############################\n{self.ID} ready!\n###############################\n")


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
        if not self.execute_trajectory_client.wait_for_server(timeout=rospy.Duration(secs=5)):
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

        def cb(progress):
            print(progress)

        self.execute_trajectory_client.send_goal(goal, feedback_cb=cb) # TODO: consider adding feedback function
        print(f"{self.ID} executing the uploaded trajectory...")

        self.execute_trajectory_client.wait_for_result()
        
        result=self.execute_trajectory_client.get_result()

        if result.success:
            print(f"{self.ID} executed the uploaded trajectory successfully!")
        else:
            print(f"{self.ID} failed to execute the uploaded trajectory!")
        return result.success








class Fleet:
    def __init__(self, config_file):
        """
        Class responsible for handling a fleet of F1/10 cars
        
        Arguments:
            - config_file(str):  Path string to the yaml file containing all the
                            configuration data needed for the setup
        
        """
        if config_file is None:
            config_file=str(Path(__file__).parents[1])+"/config/configuration.yaml"

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
        
        # OptiTrack configuration
        self.OPTITRACK_SERVER_IP=config_data["OPTITRACK_SERVER_IP"]

        # System frequency for the controllers, state observers...
        self.FREQUENCY=config_data["FREQUENCY"]

        # Load vehicle config data
        self.vehicle_data=config_data["vehicles"]

        # init list to store Car objects
        self.cars=[]

        # init GUI manager
        self.GUIManager=GUIManager()

        # set ROS parameters
        try:
            rospy.set_param("/AIMotionLab/mocap_server", self.OPTITRACK_SERVER_IP)
        except KeyError:
            print("OptiTrack server IP is not set. Vehicles expect position data stream from external ROS node!")
        
        rospy.set_param("/AIMotionLab/FREQUENCY",self.FREQUENCY)


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

    
    def init_cars(self, IDs=None):
        """
        Launches a chooser application to select vehicle's to use the initializes the control interface for them
        """
        self.drop_cars()
        
        # no ID is specified opens a dialog
        if IDs is None:
            chosen=self.choose_cars()
            for id in chosen:
                c=Car(id, self.vehicle_data[id])
                print(f"Vehicle control interface initialized for {c.ID}")
                self.cars.append(c)

        # one car ID provided
        elif isinstance(IDs, str):
            c=Car(IDs, self.vehicle_data[IDs])
            print(f"Vehicle control interface initialized for {c.ID}")
            self.cars.append(c)

        # multiple IDs provided 
        elif isinstance(IDs, list):
            for id in IDs:
                c=Car(id, self.vehicle_data[id])
                print(f"Vehicle control interface initialized for {c.ID}")
                self.cars.append(c)
        else:
            raise ValueError("Unexpected argument... IDs must be list, str or None!")

        


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
                   If not provided connect to all cars specified in condig_file
        """
        # create a list of failed interfaces to delete later
        failed=[]

        if IDs is None: # start all cars
            for car in self.cars:
                try:
                    car.launch_system()
                    car.check_steering()
                except Exception:
                    print(f"Unable establish connection to {car.ID}, deleting interface...")
                    failed.append(car)
                        
    
        else: # start only the specified cars only
            for car in self.cars:
                if car.ID in IDs:
                    try:
                        car.launch_system()
                        car.check_steering()
                    except Exception as e:
                        print(f"Unable establish connection to {car.ID}, deleting interface...")
                        print(e)
                        failed.append(car)
        
        # if any of the launchers fail delete the control interface
        for fail in failed:
            self.cars.remove(fail)
            del fail


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
        for car in self.cars:
            car.install_system(environ["ROS_MASTER_URI"])


    def keyboard_remote(self):
        """
        Control the car via keyboard

        Arguments:
            - ID(str): The ID of the car to control
        """

        # init controller
        controller=VehicleController([car.ID for car in self.cars],self.FREQUENCY)

        # connect control signal
        controller.control.connect(self.control_car)

        # execute controller
        controller.show()

        self.GUIManager.app.exec()

        controller.deleteLater()


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


class FleetTimer:
    def __init__(self, FREQUENCY):
        self.FREQUENCY=FREQUENCY

    def start(self):
        self.t0=rospy.Time.now()

    def time(self):
        return (rospy.Time.now-self.t0).to

    def sleep(self,duration):
        rospy.sleep(duration)




        






"""
    def keyboard_control(self, ID):
        
        Control the car via keyboard

        Arguments:
            - ID(str): The ID of the car to control
        
        print(f"\n###############################\nKeyboard operation for {ID}\n###############################\n")
        print("-Move forward: keyUP\n-Move backward: KeyDOWN\n-Turn left: KeyLEFT\n-Turn right: KeyRIGHT")
        print("-Increase PWM: W\n-Decrease PWM: S\n-Increase steering angle: D\n-Decrease steering angle: A")
        print("\nTO EXIT the application press ESC\n")
        print("\n###############################\n")

        # get car object
        car=next((c for c in self.cars if c.ID == ID), None)
        if ID is None:
            print(f"No car found in the fleet with the provided ID: {ID}")
            return

        # init pygame
        pygame.init()
        pgscreen=pygame.display.set_mode((1, 1))
        pygame.display.set_caption('F1/10 keyboard input')
        
        # default values
        d_var=0.07
        delta_var=0.5
        
        # countdown timer to prevent too frequent input change
        CTD=10

        running=True
        while running:
            pressed=pygame.key.get_pressed()

            if CTD: # if timer is nonzero decrease and skip input change  
                CTD-=1
            else:
            # update PWM value
                if (pressed[pygame.K_w] and pressed[pygame.K_s]):
                    pass # if both buttons are pressed do nothing
                elif pressed[pygame.K_w]:
                    CTD=10
                    d_var=d_var+0.01
                    if d_var>=0.2:
                        d_var=0.2 # contrain PWM maximum for safety
                elif pressed[pygame.K_s]:
                    CTD=10
                    d_var=d_var-0.01
                    if d_var<=0:
                        d_var=0 # do not let it go below zero

                # update steering value
                if (pressed[pygame.K_d] and pressed[pygame.K_a]):
                    pass # if both buttons are pressed do nothing
                elif pressed[pygame.K_d]:
                    CTD=10
                    delta_var=delta_var+0.01
                    if delta_var>0.45:
                        delta_var=0.6 # maximal steering angle
                elif pressed[pygame.K_a]:
                    CTD=10
                    delta_var=delta_var-0.01
                    if delta_var<0:
                        delta_var=0 # 'minimal' steering angle

            # display current values
            print(f"Current delta={round(delta_var,2)}   Current d={round(d_var,3)}", end="\r")


            # set duty cycle input
            if (pressed[pygame.K_UP] and pressed[pygame.K_DOWN]):
                d=0
            elif pressed[pygame.K_DOWN]:
                d=-d_var
            elif pressed[pygame.K_UP]:
                d=d_var
            else:
                d=0

            # set steering input
            if (pressed[pygame.K_LEFT] and pressed[pygame.K_RIGHT]):
                delta=0
            elif pressed[pygame.K_LEFT]:
                delta=-delta_var
            elif pressed[pygame.K_RIGHT]:
                delta=delta_var
            else:
                delta=0
            
            # break the loop
            if pressed[pygame.K_ESCAPE]:
                running=False
            
            # emit control signal
            car.control(d, delta)
            
            # pump events and sleep
            pygame.event.pump()
            time.sleep(0.01)

    def keyboard_control2(self, ID):
        
        Control the car via keyboard

        Arguments:
            - ID(str): The ID of the car to control
        
        print(f"\n###############################\nKeyboard operation for {ID}\n###############################\n")
        print("-Move forward: keyUP\n-Move backward: KeyDOWN\n-Turn left: KeyLEFT\n-Turn right: KeyRIGHT")
        print("-Increase PWM: W\n-Decrease PWM: S\n-Increase steering angle: D\n-Decrease steering angle: A")
        print("\nTO EXIT the application press ESC\n")
        print("\n###############################\n")

        # get car object
        car=next((c for c in self.cars if c.ID == ID), None)
        if ID is None:
            print(f"No car found in the fleet with the provided ID: {ID}")
            return

        # init pygame
        pygame.init()
        pgscreen=pygame.display.set_mode((1, 1))
        pygame.display.set_caption('F1/10 keyboard input')
        
        # default values
        d_var=0.06
        delta_var=0
        
        # countdown timer to prevent too frequent input change
        CTD=10

        running=True
        while running:
            pressed=pygame.key.get_pressed()

            if CTD: # if timer is nonzero decrease and skip input change  
                CTD-=1
            else:
            # update PWM value
                if (pressed[pygame.K_w] and pressed[pygame.K_s]):
                    pass # if both buttons are pressed do nothing
                elif pressed[pygame.K_w]:
                    #CTD=5
                    d_var=d_var+0.01
                    if d_var>=0.2:
                        d_var=0.2 # contrain PWM maximum for safety
                elif pressed[pygame.K_s]:
                    #CTD=5
                    d_var=d_var-0.01
                    if d_var<=0:
                        d_var=0 # do not let it go below zero

                # update steering value
                if (pressed[pygame.K_d] and pressed[pygame.K_a]):
                    pass # if both buttons are pressed do nothing
                elif pressed[pygame.K_d]:
                    #CTD=5
                    delta_var=delta_var+0.01
                    if delta_var>0.45:
                        delta_var=0.45 # maximal steering angle
                elif pressed[pygame.K_a]:
                    #CTD=5
                    delta_var=delta_var-0.01
                    if delta_var<-0.45:
                        delta_var=-0.45 # 'minimal' steering angle

            # display current values
            print(f"Current delta={round(delta_var,2)}   Current d={round(d_var,3)}", end="\r")


            # set duty cycle input
            if (pressed[pygame.K_UP] and pressed[pygame.K_DOWN]):
                d=0
            elif pressed[pygame.K_DOWN]:
                d=-d_var
            elif pressed[pygame.K_UP]:
                d=d_var
            else:
                d=0

            
            # break the loop
            if pressed[pygame.K_ESCAPE]:
                running=False
            
            # emit control signal
            car.control(d, delta_var)
            
            # pump events and sleep
            pygame.event.pump()
            time.sleep(0.01)


"""