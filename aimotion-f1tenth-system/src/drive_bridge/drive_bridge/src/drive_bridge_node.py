#!/usr/bin/env python

import rospy
from drive_bridge_msg.msg import InputValues
from std_msgs.msg import Float64
import yaml
import os

class DriveBridge:
    def __init__(self):
        rospy.init_node("drive_bridge", anonymous=True)

        # get ROS parameters
        with open(os.path.dirname(os.path.dirname(__file__))+"/config/config.yaml") as f:
            try:
                parameters=yaml.safe_load(f)
            except yaml.YAMLError as e:
                print("Cannot load YAML data!")

        self.angle_offset=float(parameters["STEERING_GAINS"][1])
        self.angle_gain=float(parameters["STEERING_GAINS"][0])
        self.reference_limit=float(parameters["MOTOR_LIMIT"])

        # Create Publishers
        self.delta_pub=rospy.Publisher("commands/servo/position", Float64, queue_size=1)
        self.duty_pub=rospy.Publisher("commands/motor/duty_cycle", Float64, queue_size=1)
        self.emergency_shutdown=False


    def clamp_d(self, d):
        if d<-self.reference_limit:
            d=-self.reference_limit
        elif d > self.reference_limit:
            d=self.reference_limit
        
        return d

    def send_commands(self, data):
    	if not self.emergency_shutdown:
            # rescale steering angle
            steering_angle=self.angle_offset+self.angle_gain*data.delta
        
            # publish messages
            self.delta_pub.publish(steering_angle)
            self.duty_pub.publish(self.clamp_d(data.d))
        else:
            # publish messages
            self.delta_pub.publish(self.angle_offset)
            self.duty_pub.publish(0)


    def shutdown(self):
        # publish messages
        self.delta_pub.publish(self.angle_offset)
        self.duty_pub.publish(0)

# launch main
if __name__=="__main__":
    try:
        drive_bridge=DriveBridge()
        rospy.Subscriber("control", InputValues, callback=drive_bridge.send_commands)
        rospy.on_shutdown(drive_bridge.shutdown)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass