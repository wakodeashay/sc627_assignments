#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import os,sys
import actionlib

ANG_MAX = math.pi
VEL_MAX = 0.15
od=[0,0,0,0]
lt_od = [0,0,0]
rt_od = [0,0,0]

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    #gain_ang = 1 #modify if necessary
    gain_ang = 2
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_odom(data):
    '''
    Get robot data
    '''
    global od
    od[0]=data.pose.pose.position.x
    od[1]=data.pose.pose.position.y
    od[2]=data.twist.twist.angular.z
    od[3]=data.header.stamp
    #print(data.header.stamp)
    return od

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    global lt_od
    lt_od[0]=data.pose.pose.position.x
    lt_od[1]=data.pose.pose.position.y
    lt_od[2]=data.twist.twist.angular.z
    #print('left robot')
    #print(data)
    return lt_od

def callback_right_odom(data):
    '''
    Get right robot data
    '''
    global rt_od
    rt_od[0]=data.pose.pose.position.x
    rt_od[1]=data.pose.pose.position.y
    rt_od[2]=data.twist.twist.angular.z
    #print('right robot')
    #print(data)
    return rt_od

rospy.init_node('assign4_skeleton', anonymous = True)
rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)

path2="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot2.txt"
f2 = os.open(path2,os.O_RDWR)

path3="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot3.txt"
f3 = os.open(path3,os.O_RDWR)

path4="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot4.txt"
f4 = os.open(path4,os.O_RDWR)

path5="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot5.txt"
f5 = os.open(path5,os.O_RDWR)

path6="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot6.txt"
f6 = os.open(path6,os.O_RDWR)

path7="/home/ashay/catkin_ws/src/sc627_assignments/assignment_4/bot7.txt"
f7 = os.open(path7,os.O_RDWR)


while True: #replace with balancing reached?
    #calculate v_x, v_y as per the balancing strategy
    #Make sure your velocity vector is feasible (magnitude and direction)
    #print(od)
    #convert velocity vector to linear and angular velocties using velocity_convert function given above
    left_robot_x=lt_od[0] 
    right_robot_x=rt_od[0]
    middle_robot_x=od[0]
    vel_y=0
    vel_x=(left_robot_x+right_robot_x)/2-middle_robot_x
    [v_lin,v_ang]=velocity_convert(od[0],od[1],od[2],vel_x,vel_y)
    #print(rospy.get_namespace())
    #publish the velocities below
    
    if rospy.get_namespace() == "/bot_2/":
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f2,str.encode(s)) 
    elif rospy.get_namespace() == "/bot_3/":  
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f3,str.encode(s))
    elif rospy.get_namespace() == "/bot_4/":  
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f4,str.encode(s))
    elif rospy.get_namespace() == "/bot_5/":  
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f5,str.encode(s))
    elif rospy.get_namespace() == "/bot_6/":  
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f6,str.encode(s))
    elif rospy.get_namespace() == "/bot_7/":  
        s=str(od[0])+str(',')+str(od[1])+str(',')+str(od[3])+str('\n')
        os.write(f7,str.encode(s))
    
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    #store robot path with time stamps (data available in odom topic)
    r.sleep()