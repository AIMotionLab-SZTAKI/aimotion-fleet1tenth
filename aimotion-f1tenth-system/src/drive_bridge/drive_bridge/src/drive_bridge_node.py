#!/usr/bin/env python

import rospy
from drive_bridge_msg.msg import InputValues
from std_msgs.msg import Float64


class DriveBridge:
    def __init__(self):
        rospy.init_node("drive_bridge", anonymous=True)

        # get ROS parameters
        self.angle_offset=float(rospy.get_param("~angle_offset", 0.5))
        self.angle_gain=float(rospy.get_param("~angle_gain", 1))
        self.reference_limit=float(rospy.get_param("~reference_limit", 0.25))

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