#!/usr/bin/python3
import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Given three colinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False

#check orientation of the ordered tripplet
def orientation(p, q, r):

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):

        # Clockwise orientation
        return 1
    elif (val < 0):

        # Counterclockwise orientation
        return 2
    else:

        # Colinear orientation
        return 0

#function to check crossing between hippo and hound path. A path is from one node to another node.
# The main function that returns true if the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):

    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # Special Cases

    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True

    # If none of the cases
    return False

#p1 is from hound, p2 is from hippo.
#return true if for center p1 of hound, center p2 of hippo, then collide
def check_two_circles_intersect(p1,radius1,p2,radius2):
    if pow(radius1 - radius2,2) <= (pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2)) <= pow(radius1 + radius2,2):
        return True
    else:
        return False

#it is a line segment
def check_line_intersect_circle(p1,p2,center,radius):
    ax = p1.x
    ay = p1.y
    bx = p1.x
    by = p1.y
    cx = center.x
    cy = center.y
    r = radius

    ax = ax - cx
    ay = ay - cy
    bx = bx - cx
    by = by - cy
    a = pow((bx - ax),2) + pow((by - ay),2)
    b = 2*(ax*(bx - ax) + ay*(by - ay))
    c = pow(ax,2) + pow(ay,2) - pow(r,2)
    disc = pow(b,2) - 4*a*c
    if disc <= 0:
        return False
    sqrtdisc = math.sqrt(disc)
    t1 = (-b + sqrtdisc)/(2*a)
    t2 = (-b - sqrtdisc)/(2*a)
    if (0 < t1 and t1 < 1) or (0 < t2 and t2 < 1):
        return True
    return False




# Returns true if the point p lies inside the polygon[] with n vertices
def isInsideRectanlge(polygon, p):
    # Create a point for line segment from p to infinite
    extreme = Point(math.inf, p.y)

    # Count intersections of the above line with sides of polygon
    count, i = 0, 0
    while(True):
        next = (i+1)%4
        # Check if the line segment from 'p' to 'extreme' intersects
        # with the line segment from 'polygon[i]' to 'polygon[next]'
        if doIntersect(polygon[i], polygon[next], p, extreme):
            # If the point 'p' is colinear with line segment 'i-next',
            # then check if it lies on segment. If it lies, return true,
            # otherwise false
            if orientation(polygon[i], p, polygon[next]) == 0:
                return onSegment(polygon[i], p, polygon[next])
            count = count + 1
        i = next
        if i == 0:
            break
    # Return true if count is odd, false otherwise
    return count%2 == 1

def isInsidePrlgm(p,poly):
    inside = False
    xb = poly[0].x - poly[1].x
    yb = poly[0].y - poly[1].y
    xc = poly[2].x - poly[2].x
    yc = poly[2].y - poly[2].y
    xp = p.x - poly[1].x
    yp = p.y - poly[1].y
    d = xb * yc - yb * xc
    if d != 0:
        oned = 1.0 /d
        bb = (xp * yc - xc * yp) * oned
        cc = (xb * yp - xp * yb) * oned
        inside = (bb >= 0) and (cc >= 0) and (bb <= 1) and (cc <= 1)
    return inside

def point_line_intersect_circles(m,d,center,r):
    a,b = center.x, center.y
    alpha = pow(r,2)*(1+pow(m,2)) - pow(b-m*a-d,2)
    if alpha < 0 or alpha == 0:
        print("error")
    x1,x2 = (a+b*m-d*m+math.sqrt(alpha))/(1+pow(m,2)), (a+b*m-d*m-math.sqrt(alpha))/(1+pow(m,2))
    y1,y2 = (d+a*m+b*pow(m,2)+m*math.sqrt(alpha))/(1+pow(m,2)), (d+a*m+b*pow(m,2)-m*math.sqrt(alpha))/(1+pow(m,2))
    return Point(x1,y1), Point(x2,y2)

def sgn(x):
    if x < 0:
        return -1
    else:
        return 1

def between_value(a,b,c):
    return a <= b <= c

def clic(p1,p2,center,r):
    #print("original : ",p1.x, " ", p1.y, " ", p2.x, " ", p2.y," ")
    p1x, p1y, p2x, p2y = p1.x, p1.y, p2.x, p2.y
    p1x = p1x - (center.x)
    p1y = p1y - (center.y)
    p2x = p2x - (center.x)
    p2y = p2y - (center.y)
    dx = p2x - p1x
    dy = p2y - p1y
    dr = math.sqrt(dx*dx+dy*dy)
    D = p1x*p2y - p2x*p1y
    discriminant = r*r*dr*dr - D*D
    if discriminant < 0:
        return False
    else:
        x1 = (D*dy+sgn(dy)*dx*math.sqrt(discriminant))/(dr*dr)
        x2 = (D*dy-sgn(dy)*dx*math.sqrt(discriminant))/(dr*dr)
        y1 = (-D*dx+abs(dy)*math.sqrt(discriminant))/(dr*dr)
        y2 = (-D*dx-abs(dy)*math.sqrt(discriminant))/(dr*dr)
    #print(x1," ", y1, " ", x2, " ", y2)
    #print(p1x, " ", p1y, " ", p2x, " ", p2y, " ")

    if p1x == p2x:
        if (between_value(p1y,y1,p2y) or between_value(p2y,y1,p1y) or between_value(p1y,y2,p2y) or between_value(p2y,y2,p1y)):
            return True
        return False
    elif p1y == p2y:
        if (between_value(p1x,x1,p2x) or between_value(p2x,x1,p1x) or between_value(p1x,x2,p2x) or between_value(p2x,x2,p1x)):
            return True
        return False
    else:
        if (between_value(p1x,x1,p2x) or between_value(p2x,x1,p1x) or between_value(p1x,x2,p2x) or between_value(p2x,x2,p1x)):
            return True
        return False

    return False

#check if a rectangle intersect with a stationed circle
def intersect(P,R,A,B,C,D):
    if clic(A,B,P,R) or clic(B,C,P,R) or clic(C,D,P,R) or clic(B,D,P,R) or clic(A,C,P,R) or clic(A,D,P,R):
       return True
    return False



def get_rectangle_points(p1,p2,r):
    if (p2.x - p1.x) == 0:
        #verticle movement
        A = Point(p1.x+r,p1.y)
        B = Point(p1.x-r,p1.y)
        C = Point(p2.x+r,p2.y)
        D = Point(p2.x-r,p2.y)
        return A,B,C,D
    else:
        m = (p2.y-p1.y)/(p2.x-p1.x)
        if m == 0:
            A = Point(p1.x,p1.y+r)
            B = Point(p1.x,p1.y-r)
            C = Point(p2.x,p2.y+r)
            D = Point(p2.x,p2.y-r)
            return A,B,C,D
        else:
            m1 = -1/m
            b1 = p1.y - m1*p1.x
            #print("m1 = ",m1, "b1 = ",b1)
            A, B = point_line_intersect_circles(m1,b1,p1,r)
            m2 = m1
            b2 = p2.y - m2*p2.x
            #print("m2 = ", m2, "b2 = ",b2)
            C, D = point_line_intersect_circles(m2,b2,p2,r)
            return A,B,C,D

#robot 1 is moving, while robot 2 is stationary. check if a rectangle overlap the circle or not.
def check_robot_is_moving(p1,p2,radius1,center,radius2):
    A,B,C,D = get_rectangle_points(p1,p2,radius1)
    #print("my A, B, C, D: ", A.x," ",A.y, " ", B.x, " ", B.y, " ", C.x, ' ', C.y, ' ', D.x, ' ', D.y)
    #clic(B,C,center,radius2)
    return intersect(center,radius2, A,B,C,D)

#if __name__ == '__main__':
    #p1 = Point(-1,-1)
    #p2 = Point(-2,2)
    #p3 = Point(-1.5,-1)
    #p4 = Point(-1,2)
    #if doIntersect(p1,p2,p3,p4):
    #    print('yes')
    #r1 = Point(1,1)
    #r2 = Point(2,1)
    #if check_two_circles_intersect(r1,0.5,r2,0.5):
    #    print('yes')
    #ph1 = Point(1,2)
    #ph2 = Point(5,2)
    #center = Point(3,0)
    #if check_robot_is_moving(ph1,ph2,2,center,1):
    #    print('moving Yes')
    #if isInsideRectanlge([Point(1,1),Point(1,2),Point(3,1),Point(3,2)],Point(2,1.2)):
        #print('Yes')
    #polygon = Polygon([(1,1),(1,2),(3,1),(3,2)])
    #print(polygon.contains(point(2,1.2)))
    #A,B,C,D = get_rectangle_points(Point(1,1),Point(3,3),1)
    #if check_robot_is_moving(Point(1,0),Point(2,4),1,Point(3,2),0.2):
    #    print('yes')
    #if clic(Point(-2,-2),Point(2,2),Point(2,7),2):
    #   print('YES')
