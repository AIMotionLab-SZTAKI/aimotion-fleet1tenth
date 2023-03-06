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
fleet1tenth=Fleet1tenth(config_file_path=None)

fleet1tenth.fleet.init_cars("AI_car_01")
fleet1tenth.fleet.launch_cars("AI_car_01")
c1=fleet1tenth.fleet.get_car_by_ID("AI_car_01")

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

        self.tck, self.u, *rest = splprep([X, Y], k=2, s=1, u=par)

        speed_vect=const_speed*np.ones(len(self.u))
        for i in range(50):
            speed_vect[i]=0.4+const_speed*i/100
            speed_vect[-i-1]=0.4+const_speed*i/100
        speed_vect[-1]=0
        self.speed_tck=splrep(self.u,speed_vect)
    
    def plot_traj(self):
        (x,y)=splev(self.u, self.tck)
        v=splev(self.u,self.speed_tck)
        plt.plot(x,y)
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

    


path=Path(np.array(
    [
        [-1.57, 1.54],
        [-1.25, 1.3],
        [0, 1.25],
        [0.48,1.15],
        [0.88, 0.88],
        [1.15, 0.48],
        [1.25, 0],
        [1.15, -0.48],
        [0.88, -0.88],
        [0.48, -1.15],
        [0, -1.25],
        [-0.48, -1.15],
        [-0.88, -0.88],
        [-1.15, -0.48],
        [-1.25, -0],
        [-1.15, 0.48],
        [-0.88, 0.88],
        [-0.48, 1.15],
        [0, 1.25],
        [0.48,1.15],
        [0.88, 0.88],
        [1.15, 0.48],
        [1.25, 0],
        [1.15, -0.48],
        [0.88, -0.88],
        [0.48, -1.15],
        [0, -1.25],
        [-0.48, -1.15],
        [-0.88, -0.88],
        [-1.15, -0.48],
        [-1.25, -0],
        [-1.15, 0.48],
        [-0.88, 0.88],
        [-0.48, 1.15],
        [0, 1.25],
        [0.48,1.15],
        [0.88, 0.88],
        [1.15, 0.48],
        [1.25, 0],
        [1.15, -0.48],
        [0.88, -0.88],
        [0.48, -1.15],
        [0, -1.25],
        [-0.48, -1.15],
        [-0.88, -0.88],
        [-1.15, -0.48],
        [-1.25, -0],
        [-1.15, 0.48],
        [-0.88, 0.88],
        [-0.48, 1.15],
        [0, 1.25],
        [0.48,1.15],
        [0.88, 0.88],
        [1.15, 0.48],
        [1.25, 0],
        [1.15, -0.48],
        [0.88, -0.88],
        [0.48, -1.15],
        [0, -1.5],
        [-0.5, -0.75],
        [0, 0],
    ]
), 0.8
)

x,y,theta_p=path.get_starting_data()
print(f"Starting position: {x},  {y},  {theta_p}")

path.plot_traj()

res=c1.execute_trajectory(path.tck, path.speed_tck, (0,path.length))
if not res:
    print("FAIL")