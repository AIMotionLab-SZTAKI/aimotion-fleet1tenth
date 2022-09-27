#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header
from vesc_msgs.msg import VescStateStamped
from vehicle_state_msgs.msg import VehicleStateStamped
import math


class Estimator:
    """Class for estimating velocity level information from position data"""
    def __init__(self, l_offs,filter_window, frequency, cutoff):
        # moving average filter window
        self.N=filter_window

        # Step size
        self.dt=1.0/frequency

        # lowpass filter for output
        self.alpha = self.dt/(1.0/(cutoff*2*math.pi)+self.dt)

   
        # store previous timestamp
        self.prev_time=0.0

        # marker offset
        self.l_offs=l_offs

        # X coordinate
        self.x_list=[0 for _ in range(filter_window)]

        # Y coordinate
        self.y_list=[0 for _ in range(filter_window)]

        # Heading
        self.fi_list=[0 for _ in range(filter_window)]

        # Velocity
        self.vx_filt=0
        self.vy_filt=0

        self.omega_filt=0


        # VESC callback memory
        self.d=0
        self.delta=0
        self.ERPM=0

        # create rospy Publisher
        self.state_pub=rospy.Publisher("state", VehicleStateStamped, queue_size=1)

    def newX(self,x):
        self.x_list.pop(0)
        self.x_list.append(x)

        return sum(self.x_list)/self.N

    def newY(self,y):
        self.y_list.pop(0)
        self.y_list.append(y)

        return sum(self.y_list)/self.N

    def condFi(self, newFi):
        if newFi-self.fi_list[-1]<-1.8*math.pi:
            self.fi_list=[item-2*math.pi for item in self.fi_list]
        elif newFi-self.fi_list[-1]>1.8*math.pi:
            self.fi_list=[item+2*math.pi for item in self.fi_list]



    def newFi(self,fi):
        if abs(fi-self.fi_list[-1])>1.8*math.pi:
            self.fi_list=[fi for _ in range(self.N)]
        else:
            self.fi_list.pop(0)
            self.fi_list.append(fi)

        return sum(self.fi_list)/self.N

    def getX(self):
        return sum(self.x_list)/self.N

    def getY(self):
        return sum(self.y_list)/self.N

    def getFi(self):
        return sum(self.fi_list)/self.N

    def setD(self,data):
        self.d=data.state.duty_cycle

    def setDelta(self, data):
        self.delta=data.data
        
    def setERPM(self, data):
        self.ERPM=data.state.speed
    
    def setData(self, data):
        self.setD(data)
        self.setERPM(data)
    
    def process(self,data):
        # get timestamp
        time_str=str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs).zfill(9)
        time_float=float(time_str)

        # downsapling
        dt_=time_float-self.prev_time
        if abs(dt_)<self.dt*0.98: # only update data puffers
            return

        # extract quaternion
        quaternion = np.array([data.pose.orientation.x, 
                           data.pose.orientation.y, 
                           data.pose.orientation.z, 
                           data.pose.orientation.w])


        # calculate heading angle from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)
        fi = euler[2]
        if fi<0:
            fi=fi+2*math.pi
        self.condFi(fi)
    
        # extract position in OptiTrack's global coordinates
        # use l_offs and heading angle to calculate CoM
        x = data.pose.position.x-self.l_offs*np.cos(fi)
        y = data.pose.position.y-self.l_offs*np.sin(fi)
        #x=data.pose.position.x
        #y=data.pose.position.y

        # get previous and current point w/ moving average filter
        prev_x=self.getX()
        prev_y=self.getY()
        prev_fi=self.getFi()

        x=self.newX(x)
        y=self.newY(y)
        fi=self.newFi(fi)

        # relative position from previous point
        x = x - prev_x
        y = y - prev_y

        # heading angle difference
        dfi = fi-prev_fi

        # rotate current positions by heading angle difference to eliminate the effect of steering
        x_rot = math.cos(-dfi)*x - math.sin(-dfi)*y 
        y_rot = math.sin(-dfi)*x + math.cos(-dfi)*y

        # back to the original coordinate system
        x_rot = x_rot + prev_x
        y_rot = y_rot + prev_y
        x = x + prev_x
        y = y + prev_y
       
        # rotate by original heading
        x_rot2 = math.cos(-prev_fi)*x_rot - math.sin(-prev_fi)*y_rot 
        y_rot2 = math.sin(-prev_fi)*x_rot + math.cos(-prev_fi)*y_rot

        # rotate original point by heading
        prev_x_rot = math.cos(-prev_fi)*prev_x - math.sin(-prev_fi)*prev_y
        prev_y_rot = math.sin(-prev_fi)*prev_x + math.cos(-prev_fi)*prev_y

        # obtain veloctiy by differentiation
        vx = (x_rot2 - prev_x_rot)/dt_
        vy = (y_rot2 - prev_y_rot)/dt_

        #vx_filt=vx
        #vy_filt=vy

        self.vx_filt = self.vx_filt + (self.alpha*(vx - self.vx_filt))
        self.vy_filt = self.vy_filt + (self.alpha*(vy - self.vy_filt)) 
	
	    #self.vx_filt=(x-prev_x)/dt_
	    #self.vy_filt=(y-prev_y)/dt_
        # approximate yaw rate
        
        omega = dfi/dt_
        self.omega_filt = self.omega_filt + (self.alpha*(omega-self.omega_filt))

        # construct message & publish
        msg=VehicleStateStamped()
        
        msg.header=Header() #timestamp
        msg.header.stamp=data.header.stamp
        
        msg.position_x=x #position level data
        msg.position_y=y
        msg.heading_angle=fi

        msg.velocity_x=self.vx_filt #velocity level data
        msg.velocity_y=-self.vy_filt # negate because of OptiTrack-model transition
        msg.heading_angle=fi
        msg.omega=self.omega_filt

        msg.duty_cycle=self.d
        msg.delta=self.delta
        msg.ERPM=self.ERPM

        self.state_pub.publish(msg) # publish message

        # store timestamp
        self.prev_time=time_float
 

if __name__ == '__main__':
    try:
        # Create node
        rospy.init_node('state_observer_node', anonymous=True)

        # Get ROS parames
        l_offs=rospy.get_param("~tracker_offset", 0.2225) # distance of tracked RigidBody from CoM
        frequency=rospy.get_param("~frequency", 20.0)
        cutoff=rospy.get_param("~cutoff", 2)

        estimator=Estimator(l_offs=l_offs, filter_window=10, frequency=frequency, cutoff=cutoff)
        
        # init subscribers
        rospy.Subscriber("aimotion_mocap_node/pose", PoseStamped, estimator.process)

        rospy.Subscriber("sensors/core", VescStateStamped, estimator.setData)
        rospy.Subscriber("sensors/servo_position_command", Float64, estimator.setDelta)

        rospy.loginfo("State observer initialized...")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
