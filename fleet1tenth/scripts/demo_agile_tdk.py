from scipy.interpolate import splprep, splrep, splev
import numpy as np
import matplotlib.pyplot as plt
import pickle
from fleet1tenthpy import Fleet1tenth
import time
import rospy
from vehicle_state_msgs.msg import VehicleStateStamped

class Path:
    def __init__(
        self,
        path_points,const_speed, smoothing
    ):
        self.x_points = path_points[:, 0].tolist()
        self.y_points = path_points[:, 1].tolist()

        if (
            path_points[0, 0] == path_points[-1, 0]
            and path_points[0, 1] == path_points[-1, 1]
        ):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.001)  # closed
        elif len(self.x_points):
            tck, *rest = splprep([self.x_points, self.y_points], k=1, s=0.001)  # line
        else:
            tck, *rest = splprep([self.x_points, self.y_points], k=2, s=0.001)  # curve

        u = np.arange(0, 1.001, 0.001)
        path = splev(u, tck)

        (X, Y) = path
        s = np.cumsum(np.sqrt(np.sum(np.diff(np.array((X, Y)), axis=1) ** 2, axis=0)))
        self.length = s[-1]

        par = np.linspace(0, self.length, 1001)
        par = np.reshape(par, par.size)

        self.tck, self.u, *rest = splprep([X, Y], k=2, s=smoothing, u=par)

        speed_vect=const_speed*np.ones(len(self.u))
        for i in range(50):
            speed_vect[i]=np.sign(const_speed)*0.4+(const_speed-np.sign(const_speed)*0.4)*i/50
            speed_vect[len(speed_vect)-50+i]=const_speed+(np.sign(const_speed)*0.4-const_speed)*(i)/50
        speed_vect[-1]=0
        self.speed_tck=splrep(self.u,speed_vect)
    
    def plot_traj(self):
        (x,y)=splev(self.u, self.tck)
        v=splev(self.u,self.speed_tck)
        plt.plot(x,y)
        plt.plot(self.u,v)
        plt.show()

    def set_speed(self,const_speed):
        speed_vect=const_speed*np.ones(len(self.u))
        for i in range(50):
            speed_vect[i]=np.sign(const_speed)*0.4+(const_speed-np.sign(const_speed)*0.4)*i/50
            speed_vect[len(speed_vect)-50+i]=const_speed+(np.sign(const_speed)*0.4-const_speed)*(i)/50
        speed_vect[-1]=0
        self.speed_tck=splrep(self.u,speed_vect)
        
    def get_traj(self):
        (x,y)=splev(self.u, self.tck)
        return x,y

    def get_starting_data(self):
        # position & derivatives
        s=0
        (x, y) = splev(s, self.tck)
        (x_, y_) = splev(s, self.tck, der=1)
        (x__,y__)=splev(s, self.tck,der=2)
        
        # calculate base vectors of the moving coordinate frame
        s0 = np.array(
            [x_ / np.sqrt(x_**2 + y_**2), y_ / np.sqrt(x_**2 + y_**2)]
        )
        z0 = np.array(
            [-y_ / np.sqrt(x_**2 + y_**2), x_ / np.sqrt(x_**2 + y_**2)]
        )
        theta_p=np.arctan2(s0[1],s0[0])
        return x,y,theta_p




#### AGILE MOTION DEMO ####
paths=[]
with open('test_agile_paths.pkl', 'rb') as f:
    while True:
        try:
            paths.append(pickle.load(f))
        except EOFError:
            break

paths[1].set_speed(-0.75)
paths[3].set_speed(-0.75)
paths[5].set_speed(-0.75)


fleet1tenth=Fleet1tenth(config_file_path=None)

fleet1tenth.fleet.init_cars("AI_car_01")
fleet1tenth.fleet.launch_cars("AI_car_01")
c1=fleet1tenth.fleet.get_car_by_ID("AI_car_01")
c1.init_logger("drive_logs/man", gui=False)
c1.start_logging()

#global posx, posy, heading

#def state_cb(data):
#    global posx, posy, heading
#    posx=data.position_x
#    posy=data.position_y
#    heading=data.heading_angle

#statesub=rospy.Subscriber("/AI_car_01/state", VehicleStateStamped, callback=state_cb)

# start demonstration

while 1:
    x=input("Press ENTER to start the demonstration! - Type anything in the terminal to abort")
    if x != "":
        break
    
    for path in paths:
        path.plot_traj()
        res=c1.execute_trajectory(path.tck, path.speed_tck, (0,path.length))
        if not res:
            print("Execution failed")
            break

        time.sleep(1)