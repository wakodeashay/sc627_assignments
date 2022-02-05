#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import os
#import other helper files if any
from helper import *

rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
#path_name = open("~/catkin_ws/src/assignment_1/input.txt","r")
#k=[]
#if os.path.isfile(path_name):
#f1 = open(path_name)
#k=f1.readlines()
step_size=0.1
#setting result as initial location
result = MoveXYResult()
#result.pose_final.x = int(k[0][0])
#result.pose_final.y = int(k[0][2])
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)

#goalx=int(k[1][0])
#goaly=int(k[1][2])
goalx=5
goaly=3
goal=[goalx,goaly]

#obstacle1=[[int(k[4][0]),int(k[4][2])],[int(k[5][0]),int(k[5][2])],[int(k[6][0]),int(k[6][2])]]
#obstacle2=[[int(k[8][0]),int(k[8][2])],[int(k[9][0]),int(k[9][2])],[int(k[10][0]),int(k[10][2])]]

obstacle1=[[1,2],[1,0],[3,0]]
obstacle2=[[2,3],[4,1],[5,2]]
#f1.close()
#f2 = open("output_base.txt","w")#write mode

while step_size > dist(goal,[result.pose_final.x,result.pose_final.y]) : #replace true with termination condition

    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    if step_size < min(computeDistancePointToPolygon(obstacle1,[result.pose_final.x,result.pose_final.y]), computeDistancePointToPolygon(obstacle2,[result.pose_final.x,result.pose_final.y]) ):
        wp = MoveXYGoal()
        wp.pose_dest.x = step_size*3/math.sqrt(35) + result.pose_final.x
        wp.pose_dest.y = step_size*5/math.sqrt(35)+result.pose_final.x
        wp.pose_dest.theta = math.atan(3/5) #theta is the orientation of robot in radians (0 to 2pi)

        #send waypoint to turtlebot3 via move_xy server
        client.send_goal(wp)

        client.wait_for_result()

        #getting updated robot location
        result = client.get_result()
        #f2.write(result.pose_final.x,result.pose_final.y,'\n')
        

        #write to output file (replacing the part below)
        print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
    else :
        print("Obstacle encountered stopping now")
        #f2.write(result.pose_final.x,result.pose_final.y,'\n')
#f2.close()