#! /usr/bin/env python


# Currently developed in Python2.7

from controllers import CombinedController
import rospy
#from scipy.interpolate import splprep,splrep, splev
#import numpy as np
#from drive_bridge_msg.msg import InputValues
#from vehicle_state_msgs.msg import VehicleStateStamped 


"""
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

"""


if __name__=="__main__":
    try:
        rospy.init_node("path_following_control_node", anonymous=True)
        
        # get ROS params
        freq=float(rospy.get_param("/AIMotionLab/FREQUENCY", 40.0))
        lat_gains=rospy.get_param("~lateral_gains")
        long_gains=rospy.get_param("~longitudinal_gains")
        lat_gains_reverse={"k1":[-0.0008,0.0442, -1.2247],"k2":[-0.0002,0.0191,-0.9531]} # GS TODO: move to ROS param server
        lat_gains_reverse={"k1":[-0.0124,0.11,-0.646,0.0218],"k2":[-0.0194,0.167,-1.02,0.033]} # LPV TODO: move to ROS param server

        # init controller
        controller=CombinedController(FREQUENCY=freq,projection_window=3, lateral_gains=lat_gains,lateral_gains_reverse=lat_gains_reverse,longitudinal_gains=long_gains, projection_step=0.01, look_ahead=0.2)
        
        # controller.speed_tck=
        # controller.trajectory_tck=path.tck
        # controller.s=0
        # controller.s_ref=0
        # controller.s_start=0
        # controller.s_end=path.length
        # controller.enabled=False

        # shutdown hook & spin
        rospy.on_shutdown(controller.shutdown)
        controller.spin()
    except rospy.ROSInterruptException:
        pass
