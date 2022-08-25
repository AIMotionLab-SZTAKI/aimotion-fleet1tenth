#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header
from vesc_msgs.msg import VescStateStamped
from vehicle_state_msgs.msg import VehicleStateStamped
import math
import motioncapture


class Estimator:
    """Class for estimating velocity from position data"""
    def __init__(self, optitrack_ip, car_id, l_offs,filter_window, frequency):
        # moving average filter window
        self.N=filter_window

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

        # Step size
        self.dt=1.0/frequency

        # VESC callback memory
        self.d=0
        self.delta=0
	self.ERPM=0

	# libmotioncapture
        self.mc=motioncapture.MotionCaptureOptitrack(optitrack_ip)

        # create rospy Publisher
        self.state_pub=rospy.Publisher(car_id+"/state", VehicleStateStamped, queue_size=1)

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

    def process(self):
	self.mc.waitForNextFrame()
	obj=dict(self.mc.rigidBodies.items())[car_id]

        # get timestamp
	#time=
        #time_str=str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs).zfill(9)
        #time_float=float(time_str)



        # extract quaternion
        quaternion = np.array([obj.rotation.x, 
                           obj.rotation.y, 
                           obj.rotation.z, 
                           obj.rotation.w])


        # calculate heading angle from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)
        fi = euler[2]
	if fi<0:
	    fi=fi+2*math.pi
	self.condFi(fi)
    
        # extract position in OptiTrack's global coordinates
        # use l_offs and heading angle to calculate CoM
        x = obj.position[0]-self.l_offs*np.sin(fi)
        y = obj.position[1]-self.l_offs*np.cos(fi)

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
        vx = (x_rot2 - prev_x_rot)/self.dt
        vy = (y_rot2 - prev_y_rot)/self.dt 

        vx_filt=vx
        vy_filt=vy

        #vx_filt = vx_filt + (alpha*(vx - vx_filt))
        #vy_filt = vy_filt + (alpha*(vy - vy_filt)) 

        # approximate yaw rate
        omega = dfi/self.dt


        # construct message & publish
        msg=VehicleStateStamped()
        
        msg.header=Header() #timestamp
        msg.header.stamp=rospy.Time.now()
        
        msg.position_x=x #position level data
        msg.position_y=y
        msg.velocity_x=vx_filt #velocity level data
        msg.velocity_y=vy_filt
        msg.heading_angle=fi
        msg.omega=omega

        msg.duty_cycle=self.d
        msg.delta=self.delta
	msg.ERPM=self.ERPM

        self.state_pub.publish(msg) # publish message

# Get ROS parames
car_id=rospy.get_param("/car_id", "RC_car_XX")
l_offs=rospy.get_param("/tracker_offset", 0.06) # distance of tracked RigidBody from CoM
frequency=rospy.get_param("state_observer/frequency", 20.0)
optitrack_ip=rospy.get_param("state_oberver/optitrack_ip", "192.168.1.141")

 

if __name__ == '__main__':
    try:
        # Create node
        rospy.init_node('state_observer_node', anonymous=True)

        estimator=Estimator(optitrack_ip=optitrack_ip, car_id=car_id, l_offs=l_offs, filter_window=1, frequency=frequency)
        
        # init subscribers
        rospy.Subscriber(car_id+"/sensors/core", VescStateStamped, estimator.setData)
        rospy.Subscriber(car_id+"/sensors/servo_position_command", Float64, estimator.setDelta)

        rospy.loginfo("State observer initialized for %s", car_id)
	
	# Publish vehicle state
	rate=rospy.Rate(frequency)
	while not rospy.is_shutdown():
	    estimator.process()
	    rate.sleep()

    except rospy.ROSInterruptException:
        pass




## callback functions to store control inputs
#def set_d(data):
#    global vesc_d
#    vesc_d=data.state.duty_cycle
#
#def set_delta(data):
#    global vesc_delta
#    vesc_delta=data
#
#
#def process_state(data):
#    """Estimates state variables from position data"""
#    global prev_time
#    global prev_x
#    global prev_y
#    global prev_fi
#    global vx_filt
#    global vy_filt
#
#
#    # get timestamp
#    time_str=str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs).zfill(9)
#    time_float=float(time_str)
#
#    # filter out unnecessary data sent by the VRPN client
#    if abs(time_float-prev_time)>dt*0.9: # data is valid, calculate and log
#        # get heading quaternion
#        quaternion = np.array([data.pose.orientation.x, 
#                           data.pose.orientation.y, 
#                           data.pose.orientation.z, 
#                           data.pose.orientation.w])
#
#
#        # extract heading angle
#        euler = tf.transformations.euler_from_quaternion(quaternion)
#        fi = euler[2]
#       
#        if fi < 0:
#            fi = 2*math.pi + fi
#
#        # extract position in OptiTrack's global coordinates
#        # use l_offs and heading angle to calculate CoM
#        x = data.pose.position.x-l_offs*np.sin(fi)
#        y = data.pose.position.y-l_offs*np.cos(fi)
#
#        # for the first time just save the points, do not publish
#        if prev_x is None:
#            prev_x=x
#            prev_y=y
#            prev_fi=fi
#            return
#
#        # relative position from previous point
#        x = x - prev_x
#        y = y - prev_y
#
#        # heading angle difference
#        dfi = fi-prev_fi
#
#        # rotate current positions by heading angle difference to eliminate the effect of steering
#        x_rot = math.cos(-dfi)*x - math.sin(-dfi)*y 
#        y_rot = math.sin(-dfi)*x + math.cos(-dfi)*y
#
#        # back to the original coordinate system
#        x_rot = x_rot + prev_x
#        y_rot = y_rot + prev_y
#        x = x + prev_x
#        y = y + prev_y
#       
#        # rotate by original heading
#        x_rot2 = math.cos(-prev_fi)*x_rot - math.sin(-prev_fi)*y_rot 
#        y_rot2 = math.sin(-prev_fi)*x_rot + math.cos(-prev_fi)*y_rot
#
#        # rotate original point by heading
#        prev_x_rot = math.cos(-prev_fi)*prev_x - math.sin(-prev_fi)*prev_y
#        prev_y_rot = math.sin(-prev_fi)*prev_x + math.cos(-prev_fi)*prev_y
#
#        # obtain veloctiy by differentiation
#        vx = (x_rot2 - prev_x_rot)/dt
#        vy = (y_rot2 - prev_y_rot)/dt 
#
#        vx_filt = vx_filt + (alpha*(vx - vx_filt))
#        vy_filt = vy_filt + (alpha*(vy - vy_filt)) 
#
#        # approximate yaw rate
#        omega = (fi - prev_fi)/dt
#
#        # calculate sideslip angle
#        # sideslip = np.arctan(vy_filt/vx_filt)
#
#
#        # update memory
#        prev_x=x
#        prev_y=y
#        prev_fi=fi
#
#        # construct message & publish
#        msg=VehicleStateStamped()
#        
#        msg.header=Header() #timestamp
#        msg.header.stamp=rospy.Time.now()
#        
#        msg.position_x=x #position level data
#        msg.position_y=y
#        msg.velocity_x=vx_filt #velocity level data
#        msg.velocity_y=vy_filt
#        msg.heading_angle=fi
#        msg.omega=omega
#
#        msg.duty_cycle=vesc_d
#        msg.delta=vesc_delta
#
#        state_pub.publish(msg) # publish message