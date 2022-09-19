#! /usr/bin/env python


# Currently developed in Python2.7

from control.scripts.controllers import Controller
import rospy

if __name__=="__main__":
    try:
        rospy.init_node("aimotion_control_node", anonymous=True)
        controller=FeedbackController()
        rospy.on_shutdown(controller.shutdown())
        controller.spin()
    except rospy.ROSInterruptException:
        pass
