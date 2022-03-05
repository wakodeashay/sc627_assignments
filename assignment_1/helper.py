#!/usr/bin/env python3
from sympy import Point, Polygon
import math
#def dist(p1,p2):
#    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def computeLineThroughTwoPoints(p1,p2):
    d= math.dist(p1,p2)
    if d < 10**(-2) :
        #print("Same point, give different points")
        return [0,0,0]
    else :
        return [(p2[1]-p1[1])/d,(p1[0]-p2[0])/d,(p1[1]*p2[0]-p1[0]*p2[1])/d]

def computeDistancePointToLine(p1,p2,q):
    [a,b,c] = computeLineThroughTwoPoints(p1,p2)
    return math.abs(a*q[0]+b*q[1]+c)

def computeDistancePointToSegment(p1,p2,q):
    d1=math.dist(p1,p2)
    d2=math.dist(p1,q)
    d3=math.dist(p2,q)
    if d3**2 > d1**2+d2**2:
        w=0
    elif d2**2 > d1**2+d3**2:
        w=2
    elif d1**2 > d2**2+d3**2:
        w=1
    return (computeDistancePointToLine(p1,p2,q),w)

def computeDistancePointToPolygon(P,q):
    p1, p2, p3 = map(Point, [(P[0][0],P[1][0]),(P[0][1],P[1][1]),(P[0][2],P[1][2])])
    # creating polygon using Polygon()
    poly = Polygon(p1, p2, p3)
    return poly.distance(Point(q[0],q[1]))
