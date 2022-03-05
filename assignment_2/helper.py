from sympy import Point, Polygon
from sympy.geometry import Point, Circle
import math
import numpy as np
def computeDistancePointToPolygon(P,q):
    p1, p2, p3 = map(Point, [(P[0][0],P[1][0]),(P[0][1],P[1][1]),(P[0][2],P[1][2])])
    poly = Polygon(p1, p2, p3)
    return poly.distance(Point(q[0],q[1]))

def attractive_force(current_location,goal_location):
    c=np.array(current_location)
    g=np.array(goal_location)
    d=math.dist(c,g)
    if d <= 2:
        f=-0.8*d
    else :
        f=-1.6
    p=g-c
    norm = np.linalg.norm(p)
    p=p/norm
    return -f*p
def repulsive_force_mag(distance):
    if distance <= 2:
        return float(1.2/distance**3-0.4/distance**2)
    else:
        return 0 

def repulsive_force(current_location,obstacle):
    c=np.array(current_location)
    P=obstacle
    p1, p2, p3 = map(Point, [(P[0][0],P[1][0]),(P[0][1],P[1][1]),(P[0][2],P[1][2])])
    # creating polygon using Polygon()
    poly = Polygon(p1, p2, p3)
    dist=computeDistancePointToPolygon(P,c)
    mag=repulsive_force_mag(dist)
    circle = Circle(Point(c[0],c[1]),dist)
    Intersection = poly.intersection(circle)
    point_on_polygon=[Intersection[0][0],Intersection[0][1]]
    pop=np.array(point_on_polygon)
    p=c-pop
    p=p/dist
    return mag*p

