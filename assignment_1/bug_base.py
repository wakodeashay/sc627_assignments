#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import os,sys
import numpy as np
#import other helper files if any
from helper import *
import math
rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# Reading file
# File path 
path = "/home/ashay/catkin_ws/src/sc627_assignments/assignment_1/input.txt" 
fd = os.open(path, os.O_RDONLY)  
n = 50
readBytes = os.read(fd,n)
text=str(readBytes)
start_x=int(text[2])
start_y=int(text[4])
goal_x=int(text[7])
goal_y=int(text[9])
step_size=float(text[12:15])
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start_x
result.pose_final.y = start_y
result.pose_final.theta = 0 #in radians (0 to 2pi)
# Defining Obstacles 
obstacle1_x=[int(text[19]),int(text[24]),int(text[29])]
obstacle1_y=[int(text[21]),int(text[26]),int(text[31])]
obstacle2_x=[int(text[36]),int(text[41]),int(text[46])]
obstacle2_y=[int(text[38]),int(text[43]),int(text[48])]
obstacle1=[obstacle1_x,obstacle1_y]
obstacle2=[obstacle2_x,obstacle2_y]
os.close(fd)

path1="/home/ashay/catkin_ws/src/sc627_assignments/assignment_1/output_base.txt"
f = os.open(path1,os.O_RDWR)


# Writing text
s=str(start_x)+str(',')+str(start_y)+str('\n')

#ret = os.write(f,str.encode(s))

while step_size < math.dist([goal_x,goal_y],[result.pose_final.x,result.pose_final.y]) : #replace true with termination condition
    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    if step_size < min(float(computeDistancePointToPolygon(obstacle1,[result.pose_final.x,result.pose_final.y])),float(computeDistancePointToPolygon(obstacle2,[result.pose_final.x,result.pose_final.y]))):
        wp = MoveXYGoal()
        wp.pose_dest.x = step_size*goal_x/math.sqrt(goal_x**2+goal_y**2) + result.pose_final.x
        wp.pose_dest.y = step_size*goal_y/math.sqrt(goal_x**2+goal_y**2) + result.pose_final.y
        #wp.pose_dest.theta = math.atan(3,5) #theta is the orientation of robot in radians (0 to 2pi)
        wp.pose_dest.theta = math.atan2(goal_y,goal_x)
        #send waypoint to turtlebot3 via move_xy server
        client.send_goal(wp)
        client.wait_for_result()
        #getting updated robot location
        result = client.get_result()
        #f2.write(result.pose_final.x,result.pose_final.y,'\n')
        s=str(result.pose_final.x)+str(',')+str(result.pose_final.y)+str('\n')
        os.write(f,str.encode(s))
        #write to output file (replacing the part below)
        print(result.pose_final.x,result.pose_final.y,result.pose_final.theta)
    else :
        s=str(result.pose_final.x)+str(',')+str(result.pose_final.y)+str('\n')
        # writing output to output_base file
        os.write(f,str.encode(s))
        print(result.pose_final.x,result.pose_final.y,result.pose_final.theta)
        print("Obstacle encountered stopping now")
        break
# closing the ouput file
os.close(f)