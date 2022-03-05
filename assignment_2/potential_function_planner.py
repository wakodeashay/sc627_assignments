#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import os,sys
import math
import numpy as np
#import other helper files if any
from helper import *
#import other helper files if any
rospy.init_node('test', anonymous= True)
#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
path = "/home/ashay/catkin_ws/src/sc627_assignments/assignment_2/input.txt" 
fd = os.open(path, os.O_RDONLY)  
n = 50
readBytes = os.read(fd,n)
text=str(readBytes)
start_x=int(text[2])
start_y=int(text[4])
goal_x=int(text[7])
goal_y=int(text[9])
step_size=float(text[12:15])
print(step_size)
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)
# Defining obstacle
obstacle1_x=[int(text[19]),int(text[24]),int(text[29])]
obstacle1_y=[int(text[21]),int(text[26]),int(text[31])]
obstacle2_x=[int(text[36]),int(text[41]),int(text[46])]
obstacle2_y=[int(text[38]),int(text[43]),int(text[48])]
obstacle1=[obstacle1_x,obstacle1_y]
obstacle2=[obstacle2_x,obstacle2_y]
os.close(fd)

path1="/home/ashay/catkin_ws/src/sc627_assignments/assignment_2/output.txt"
f = os.open(path1,os.O_RDWR)
# Writing text
s=str(start_x)+str(',')+str(start_y)+str('\n')
print(result.pose_final.x,result.pose_final.y,result.pose_final.theta)

while step_size < math.dist([goal_x,goal_y],[result.pose_final.x,result.pose_final.y]): #replace true with termination condition
    k=attractive_force([result.pose_final.x,result.pose_final.y],[goal_x,goal_y])+repulsive_force([result.pose_final.x,result.pose_final.y],obstacle1)+repulsive_force([result.pose_final.x,result.pose_final.y],obstacle2)
    #determine waypoint based on your algo
    j=np.array(k)
    jnorm = math.sqrt(j[0]**2+j[1]**2)
    #this is a dummy waypoint (replace the part below)
    wp = MoveXYGoal()
    wp.pose_dest.x = 2*step_size*j[0]/jnorm
    wp.pose_dest.y = 2*step_size*j[1]/jnorm
    wp.pose_dest.theta = math.atan2(j[1],j[0]) #theta is the orientation of robot in radians (0 to 2pi)
    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()
    #getting updated robot location
    result = client.get_result()
    s=str(result.pose_final.x)+str(',')+str(result.pose_final.y)+str('\n')
    os.write(f,str.encode(s))
    #write to output file (replacing the part below)
    print(result.pose_final.x,result.pose_final.y,result.pose_final.theta)
print('reached the goal')
os.close(f)