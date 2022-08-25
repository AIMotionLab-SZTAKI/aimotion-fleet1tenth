#!/usr/bin/env python3
import rospy
from vehicle_state_msgs.msg import VehicleStateStamped
from os.path import expanduser
from time import gmtime, strftime


# get ROS params
pos_bool=rospy.get_param("state_logger_node/position_level", True)
vel_bool=rospy.get_param("state_logger_node/velocity_level", True)
inp_bool=rospy.get_param("state_logger_node/inputs", True)
filename=rospy.get_param("state_logger_node/filename", "state")
car_id=rospy.get_param("/car_id", "RC_car_XX")


filename_full=strftime(expanduser("~")+"/drive_logs/"+filename+"_state_%Y-%m-%d-%H-%M-%S", gmtime())+".csv"
file=open(filename_full, "w")


def log_to_file(data):
    time=float(str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs).zfill(9))
    
    file.write(f"{time}")

    # position level data
    if pos_bool:
        file.write(f",{data.position_x},{data.position_y},{data.heading_angle}")

    if vel_bool:
        file.write(f",{data.ERPM},{data.velocity_x},{data.velocity_y},{data.omega}")

    if inp_bool:
        file.write(f",{data.duty_cycle},{data.delta}")

    file.write("\n")

    
def close_file():
    file.close()


if __name__ == '__main__':
    try:
        # Create node
        rospy.init_node('state_logger_node', anonymous=True)

        # ensure proper file closure
        rospy.on_shutdown(close_file)
        
        # init subscribers
        rospy.Subscriber(car_id+"/state", VehicleStateStamped, log_to_file)

        
        rospy.loginfo(f"State logger for {car_id} initialized... Logging to {filename_full}")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass