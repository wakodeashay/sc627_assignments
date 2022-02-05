#!/usr/bin/env python3

import math
def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def computeLineThroughTwoPoints(p1,p2):
    d= dist(p1,p2)
    if d < 10**(-8) :
        print("Same point, give different points")
    else :
        return [(p2[1]-p1[1])/d,(p1[0]-p2[0])/d,(p1[1]*p2[0]-p1[0]*p2[1])/d]

def computeDistancePointToLine(p1,p2,q):
    [a,b,c]=computeLineThroughTwoPoints(p1,p2)
    return math.abs(a*q[0]+b*q[1]+c)

def computeDistancePointToSegment(p1,p2,q):
    d1=dist(p1,p2)
    d2=dist(p1,q)
    d3=dist(p2,q)
    if d3**2 > d1**2+d2**2:
        w=0
    elif d2**2 > d1**2+d3**2:
        w=2
    elif d1**2 > d2**2+d3**2:
        w=1
    return (computeDistancePointToLine(p1,p2,q),w)

def computeDistancePointToPolygon(P,q):
    l=[]
    n=len(P)
    for i in range(n):
        l.append(computeDistancePointToLine(P[(i)%n],P[(i+1)%n],q)) 
    return min(l)

def computeTangentVectorToPolygon(P,q):

    return 