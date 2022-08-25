#!/usr/bin/env python3
import rospy
import pygame
from drive_bridge_msg.msg import InputValues

### DEFINE VALUES
duty_cycle=float(rospy.get_param("/keyboard_controller/duty_max", 0.05))
delta=float(rospy.get_param("/keyboard_controller/delta_max", 0.1))
car_id=str(rospy.get_param("/car_id", "RC_car_XX"))



class DrivePublisher(object):
    """Publishes drive messages for the drive brifge on the F1/10 platform"""

    def __init__(self):
        self.pub = rospy.Publisher(car_id + "/control", InputValues, queue_size=1)


    def update(self, event):
        msg=InputValues()

        # get pressed keys
        pressed=pygame.key.get_pressed()

        # set duty cycle input
        if (pressed[pygame.K_UP] and pressed[pygame.K_DOWN]):
            msg.d=0
        elif pressed[pygame.K_DOWN]:
            msg.d=-duty_cycle
        elif pressed[pygame.K_UP]:
            msg.d=duty_cycle
        else:
            msg.d=0

        # set steering input
        if (pressed[pygame.K_LEFT] and pressed[pygame.K_RIGHT]):
            msg.delta=0
        elif pressed[pygame.K_LEFT]:
            msg.delta=-delta
        elif pressed[pygame.K_RIGHT]:
            msg.delta=delta
        else:
            msg.delta=0

        # Publish msgs
        self.pub.publish(msg)
        #rospy.loginfo("Publishing message: %s", msg)
        pygame.event.pump()
	
        
if __name__=='__main__':
    pygame.init()
    pgscreen=pygame.display.set_mode((1, 1))
    pygame.display.set_caption('F1/10 keyboard input')
    rospy.init_node('keyboad_controller')
    freq = float(rospy.get_param('freq', 100))
    rospy.Timer(rospy.Duration(1.0/freq), DrivePublisher().update)
    rospy.spin()
