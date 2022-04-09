#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import os,sys
import actionlib
import random
import numpy as np

random.seed(5)
ANG_MAX = math.pi/18
#ANG_MAX = math.pi
VEL_MAX = 0.15
Obstacle_data = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]] 
od=[0,0,0,0,0,0] 
obstacle_dia = 0.3
goal=[5,0]

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''
    # gain_ang = 1
    gain_ang = 1.5 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    global Obstacle_data
    for i,j in zip(data.obstacles, range(len(data.obstacles))):
        Obstacle_data[j][0]=i.obs
        Obstacle_data[j][1]=i.pose_x
        Obstacle_data[j][2]=i.pose_y
        Obstacle_data[j][3]=i.vel_x
        Obstacle_data[j][4]=i.vel_y
    return Obstacle_data


def callback_odom(data):
    '''
    Get robot data
    '''
    #print(data)
    global od
    od[0]=data.twist.twist.linear.x
    od[1]=data.twist.twist.linear.y
    od[2]=data.twist.twist.angular.z
    od[3]=data.pose.pose.position.x
    od[4]=data.pose.pose.position.y
    od[5]=data.header.stamp
    return od

def collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs_x,obs_y,obs_vx,obs_vy):
    d= math.dist([robot_x,robot_y],[obs_x,obs_y])
    theta=math.atan2(obstacle_dia,d)
    phi = math.atan2(obs_y-robot_y,obs_x-robot_x)
    x=obs_vx-robot_vx
    y=obs_vy-robot_vy
    return [x,y,theta,phi]

def check_inside(xt,yt,K):
    x=K[0]
    y=K[1]
    theta=K[2]
    phi=K[3]
    r=math.atan2(yt-y,xt-x)
    t=r-phi
    if abs(t) <= theta :
        return 0
    else :
        return 1


rospy.init_node('assign3_skeleton', anonymous = True)
rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed

pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)

path="/home/ashay/catkin_ws/src/sc627_assignments/assignment_3/xy.txt"
f = os.open(path,os.O_RDWR)

path1="/home/ashay/catkin_ws/src/sc627_assignments/assignment_3/xtime.txt"
f1 = os.open(path1,os.O_RDWR)

while math.dist([od[3],od[4]],goal)>0.3: 
   obs1_x=Obstacle_data[0][1]
   obs1_y=Obstacle_data[0][2]
   obs1_vx=Obstacle_data[0][3]
   obs1_vy=Obstacle_data[0][4]

   obs2_x=Obstacle_data[1][1]
   obs2_y=Obstacle_data[1][2]
   obs2_vx=Obstacle_data[1][3]
   obs2_vy=Obstacle_data[1][4]

   obs3_x=Obstacle_data[2][1]
   obs3_y=Obstacle_data[2][2]
   obs3_vx=Obstacle_data[2][3]
   obs3_vy=Obstacle_data[2][4]

   robot_x=  od[3]
   robot_y=  od[4]
   robot_vx= od[0]
   robot_vy= od[1]
   robot_theta=od[2]
   time_stamp=od[5]

   p1=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs1_x,obs1_y,obs1_vx,obs1_vy))
   p2=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs2_x,obs2_y,obs2_vx,obs2_vy))
   p3=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs3_x,obs3_y,obs3_vx,obs3_vy))
   
   # Collision Cone algorithm
   while p1 and p2 and p3 !=1  : 
    vr=0.15*np.random.random()
    heading_angle=math.atan2(goal[1]-robot_y,goal[0]-robot_x)
    # using 3-sigma rule to find standard deviation
    stan_dev=ANG_MAX/(3/2)
    rand_heading_angle=np.random.normal(heading_angle,stan_dev)
    vx=vr*math.cos(rand_heading_angle)
    vy=vr*math.sin(rand_heading_angle)
    #vx=vr*math.cos(heading_angle)
    #vy=vr*math.sin(heading_angle)
    robot_vx=vx
    robot_vy=vy
    p1=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs1_x,obs1_y,obs1_vx,obs1_vy))
    p2=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs2_x,obs2_y,obs2_vx,obs2_vy))
    p3=check_inside(robot_vx,robot_vy,collision_cone(robot_x,robot_y,robot_vx,robot_vy,obs3_x,obs3_y,obs3_vx,obs3_vy))
    
   #convert velocity vector to linear and angular velocties using velocity_convert function given above
   [v_lin,v_ang]=velocity_convert(robot_x, robot_y, robot_theta, robot_vx, robot_vy)
   #publish the velocities below
   vel_msg = Twist()
   vel_msg.linear.x = v_lin
   vel_msg.angular.z = v_ang
   pub_vel.publish(vel_msg)
   # writing to xy.txt
   s=str(robot_x)+str(',')+str(robot_y)+str('\n')
   os.write(f,str.encode(s)) 
   # writing to xtime.txt
   s1=str(robot_x)+str(',')+str(time_stamp)+str('\n')
   os.write(f1,str.encode(s1))
   r.sleep()

print("Reached the goal")


