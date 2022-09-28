#! /usr/bin/env python


# Currently developed in Python2.7

from controllers import CombinedController
import rospy
from scipy.interpolate import splprep,splrep, splev
import numpy as np
from drive_bridge_msg.msg import InputValues
from vehicle_state_msgs.msg import VehicleStateStamped 

###ONLY FOR TESTING !REMOVE!###
class Path:
    def __init__(
        self,
        path_points,
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

        self.tck, self.u, *rest = splprep([X, Y], k=2, s=0.001, u=par)


path=Path(np.array(
    [
        [0, 0],
        [1, 1],
        [2, 2],
        [3, 2],
        [4, 1],
        [4.5, 0],
        [4, -1],
        [3, -2],
        [2, -2],
        [1, -1],
        [0, 0],
        [-1, 1],
        [-2, 2],
        [-3, 2],
        [-4, 1],
        [-4.5, 0],
        [-4, -2],
        [-3, -2],
        [-2, -2],
        [-1, -1],
        [0, 0],
    ]
)
)




if __name__=="__main__":
    try:
        rospy.init_node("aimotion_control_node", anonymous=True)
        controller=CombinedController(FREQUENCY=float(rospy.get_param("~FREQUENCY", 20)),projection_window=3, projection_step=0.01)
        #controller.speed_tck=
        controller.trajectory_tck=path.tck
        controller.s=0
        controller.s_ref=0
        controller.s_start=0
        controller.s_end=path.length
        controller.enabled=True
        rospy.on_shutdown(controller.shutdown)
        controller.spin()
    except rospy.ROSInterruptException:
        pass
