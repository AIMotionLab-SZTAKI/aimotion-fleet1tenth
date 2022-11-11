import rospy
from vehicle_state_msgs.msg import VehicleStateStamped
from .visualization import Visualization
import atexit
class PlaybackManager:
    def __init__(self, car_IDs):
        """
        Class responsible for the management of the simulation

        Arguments:
            - car_IDs(str,list): Unique indentifier of the simulated vehicles
        """


        self.car_IDs=car_IDs
        self.car_logs=[]

        for ID in self.car_IDs:
            l=CarLogs(ID)
            self.car_logs.append(l)

        self.gui=Visualization(self.car_IDs)

        # execute gui at the end of the script
        atexit.register(self.show_gui)

    def show_gui(self):
        self.gui.show()



class CarLogs:
    def __init__(self, ID):
        """
        Class to store logged vehicle data

        Arguments:
            - ID(str): Unique identifier of the logged vehicle
        """
        
        self.ID=ID

        # empty lists for the incoming data
        self.t=[]

        self.x=[]
        self.y=[]
        self.phi=[]

        self.v_xi=[]
        self.v_eta=[]
        self.omega=[]

        self.d=[]
        self.delta=[]

        self.sub=rospy.Subscriber("/"+self.ID+"/state", VehicleStateStamped, callback=self._log_callback)


    def _log_callback(self, data):
        time=float(str(data.header.stamp.secs)+str(data.header.stamp.nsecs).zfill(9))

        self.t.append(time)

        self.x.append(data.position_x)
        self.y.append(data.position_y)
        self.phi.append(data.heading_angle)
        self.v_xi.append(data.velocity_x)
        self.v_eta.append(data.velocity_y)
        self.omega.append(data.omega)
        self.d.append(data.duty_cycle)
        self.delta.append(data.delta)

    def get_position_data(self):
        return self.x, self.y, self.phi

    def get_velocity_data(self):
        return self.v_xi, self.v_eta, self.omega

    def get_input_data(self):
        return self.d, self.delta

    def get_time(self):
        return self.t