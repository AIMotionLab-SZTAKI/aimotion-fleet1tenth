from re import S
import matplotlib.pyplot as plt
from fleet1tenthpy import Fleet1tenth
import time
from scipy.interpolate import splrep, splprep, splev
import numpy as np

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



###ONLY FOR TESTING !REMOVE!###
class Path:
    def __init__(
        self,
        path_points,const_speed
    ):
        self.x_points = path_points[:, 0].tolist()
        self.y_points = path_points[:, 1].tolist()

        if (
            path_points[0, 0] == path_points[-1, 0]
            and path_points[0, 1] == path_points[-1, 1]
        ):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.0001)  # closed
        elif len(self.x_points):
            tck, *rest = splprep([self.x_points, self.y_points], k=1, s=0.0001)  # line
        else:
            tck, *rest = splprep([self.x_points, self.y_points], k=2, s=0.0001)  # curve

        u = np.arange(0, 1.001, 0.001)
        path = splev(u, tck)

        (X, Y) = path
        s = np.cumsum(np.sqrt(np.sum(np.diff(np.array((X, Y)), axis=1) ** 2, axis=0)))
        self.length = s[-1]

        par = np.linspace(0, self.length, 1001)
        par = np.reshape(par, par.size)

        self.tck, self.u, *rest = splprep([X, Y], k=2, s=0.1, u=par)

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
        plt.plot(self.x_points,self.y_points, "bo")
        plt.grid()
        plt.plot(self.u,v)
        plt.show()

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

    
### define path points

start_p=np.array([1.78,-0.22])
end_p=np.array([1.9,0.3])
rounds=9

cx=0
cy=0
r=1
points=np.zeros((12,2))


for i in range(0,12):
    points[i][0]=cx+r*np.cos(2*np.pi*i/12)
    points[i][1]=cy+r*np.sin(2*np.pi*i/12)

points=np.flip(points, axis=0)


#alloc array for all the path points
all_path=np.zeros((12*rounds+5,2))
all_path[0,:]=start_p
all_path[1,:]=np.array([start_p[0]-0.10, start_p[1]-0.0])
all_path[2,:]=np.array([start_p[0]-0.20, start_p[1]-0.0])
all_path[3,:]=np.array([start_p[0]-0.30, start_p[1]-0.05])


for i in range(rounds):
    all_path[i*12+4:i*12+16,:]=points



all_path[-1,:]=end_p
all_path[-2,:]=np.array([end_p[0]-0.2, end_p[1]])

path=Path(all_path,const_speed=0.9
)

x,y,theta_p=path.get_starting_data()
print(f"Starting position: {x},  {y},  {theta_p}")

fleet1tenth=Fleet1tenth(config_file_path=None)

fleet1tenth.fleet.init_cars("RC_car_02")
fleet1tenth.fleet.launch_cars("RC_car_02")
c1=fleet1tenth.fleet.get_car_by_ID("RC_car_02")

path.plot_traj()


res=c1.execute_trajectory(path.tck, path.speed_tck, (0,path.length))