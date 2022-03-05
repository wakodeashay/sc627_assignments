from sympy import Point, Polygon
from sympy.geometry import Point, Circle
c1 = Circle(Point(0, 0), 1)
# creating points using Point()
p11, p12, p13 = map(Point, [(1,0), (3, 0), (1,2)])
# creating polygon using Polygon()
poly1 = Polygon(p11, p12, p13)
# using distance()
#shortestDistance = poly.distance(Point(3, 5))
isIntersection = poly1.intersection(c1)
print(isIntersection[0][1])



