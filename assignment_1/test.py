#!/usr/bin/env python3
from sympy import Point, Polygon
import math
import numpy as np
def computeDistancePointToPolygon(P,q):
    p1, p2, p3 = map(Point, [(P[0][0],P[1][0]),(P[0][1],P[1][1]),(P[0][2],P[1][2])])
    poly = Polygon(p1, p2, p3)
    return poly.distance(Point(q[0],q[1]))
q=np.array([3,0])
P=[[1,1,2],[3,4,5]]
print(q[0])