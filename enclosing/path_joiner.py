# ## Source: http://stackoverflow.com/questions/2573997/reduce-number-of-points-in-line
import math

from shapely.geometry import LineString
from sympy.functions import sign
import numpy as np


######## This must be in polygon file ##############
def euclidean_distance(p1, p2):
    """
    Compute euclidean distance for two points
    :param p1:
    :param p2:
    :return:
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    # Magnitude. Coulomb law.
    return math.sqrt(dx ** 2 + dy ** 2)


def side_of_line(l, p):
    """
    Line from (x1, y1) to (x2,y2) and point (px, py)
    return 1 or -1 depending on the side of the line
    """
    x1, y1 = l[0]
    x2, y2 = l[1]
    px, py = p
    return sign((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1))


def polyline_closes_perpend(points):
    """
    Identify the cycle beginning for the last point
    :param perp_theta: perpendicular angle  to the last point.
    :param points: array of objects of PointX
    :param ddd: distance to the fist point ddd>0.
    :return -1 if do not close, or i if close in point i.
    
           
    1. create a perpendicular line to the tangent of the anomaly.
    2. use the line l to devide the workspace in two areas A and B.
     line from $A=(x_1,y_1)$ to $B=(x_2,y_2)$ a point $P=(x,y) $
         %http://math.stackexchange.com/questions/274712/calculate-on-which-side-of-straign-line-is-dot-located
        $$
        d = (x-x_1)(y_2-y_1)-(y-y_1)(x_2-x_1)
        $$
    3. identify if the second point is in A or B.
    4. if the second point is in B, the intersection must be from A to B.
        if it is in A, then from B to A.
        - evaluate only the points in the contrary area
    """
    n = len(points)
    # Minimum of three points to define a polygon.
    if n < 3:
        return None

    ## Two last points
    lp1x, lp1y = points[-1]
    lp2x, lp2y = points[-2]

    # Perpendicular angle
    perp_theta = math.atan2(lp1y - lp2y, lp1x - lp2x) + math.pi / 2.

    # last point
    lp = points[-1]

    # perpendicular line
    ddd = 100000
    x1 = lp[0] - ddd * math.cos(perp_theta)
    y1 = lp[1] - ddd * math.sin(perp_theta)
    x2 = lp[0] + ddd * math.cos(perp_theta)
    y2 = lp[1] + ddd * math.sin(perp_theta)

    perp_line = [(x1, y1), (x2, y2)]

    # Side of the second point
    side = side_of_line(perp_line, (lp2x, lp2y))

    # Analize the path backwards.
    # for n-3 to 1
    for i in range(n - 3, 0, -1):

        # line segment
        p1 = points[i]
        p2 = points[i - 1]

        # side of p1
        p1_side = side_of_line(perp_line, p1)
        # p1 in the wrong side
        if p1_side == side:
            # print "-",i, p1_side
            continue

        # side of p2
        p2_side = side_of_line(perp_line, p2)
        # print "p2=",i, p2_side
        # p2 in the good side
        if p2_side == side or p2_side == 0:
            # print "good"
            # print i, [points[-1], points[-2]]
            return i

    return None


def perpendicular_line(p1, p2, ddd=1):
    """
    Perpendicular line with respect to two points.
    The perpendicular will be referenced to p1.
    p1: point 1
    p2: point 2
    ddd: lenght of the perpendicular line
    """
    ## Two last points
    lp1x, lp1y = p1
    lp2x, lp2y = p2

    # Perpendicular angle
    perp_theta = math.atan2(lp1y - lp2y, lp1x - lp2x) + math.pi / 2.

    # last point
    lp = p1

    # perpendicular line
    x1 = lp[0] - ddd * math.cos(perp_theta)
    y1 = lp[1] - ddd * math.sin(perp_theta)
    x2 = lp[0] + ddd * math.cos(perp_theta)
    y2 = lp[1] + ddd * math.sin(perp_theta)

    return (x1, y1), (x2, y2)


def new_data_path(i, path, atime):
    """
    new point in the intersection. 
    It creates a perpendicular line from the i-point in path.
    and returns the intersection between the perpendicular and the path.
    """
    ########## Intersection point #########
    # segment of interest.
    seg = LineString([path[i], path[i - 1]])
    # perpendicular segment
    p1, p2 = perpendicular_line(path[-1], path[-2], ddd=100000)
    per_seg = LineString([p1, p2])

    inter = per_seg.intersection(seg)
    ## intersection    
    if inter.type == 'Point':
        intersection = inter.coords[0]
    elif inter.type == 'MultiPoint':
        intersection = inter[0].coords[0]
    elif inter.type == 'GeometryCollection':
        print "---NO INTERSECTION ", [path[i], path[i - 1]], i
        # intersection = inter[0].coords[0]
    else:
        intersection = None
        print "Unknown type", inter.type

    ######## Time ##########
    ## seg is divided in two elements: seg1 and seg2
    seg1 = LineString([path[i], intersection])
    seg2 = LineString([intersection, path[i - 1]])
    d1 = seg1.length
    d = seg.length
    seg.length, seg1.length, seg2.length
    ## times
    t1 = atime[i]
    t2 = atime[i - 1]

    # new time
    newt = t1 + (d1 / d) * (t2 - t1)

    return intersection, newt


def insert_in_datapath(i, newpoint, newtime, atime, path):
    """
    Add point and time in the i position of the arrays.
    """
    mtime = np.hstack((atime[:i], [newtime], atime[i:]))
    mpath = np.vstack((path[:i], newpoint, path[i:]))

    return mtime, mpath


def cutting_path(path, atime):
    """
    Modify the path to add new points.
    """
    i = polyline_closes_perpend(path)
    intersection, newt = new_data_path(i, path, atime)
    mtime, mpath = insert_in_datapath(i, intersection, newt, atime, path)

    j = len(mpath)
    # recentst is the first one
    polygons = []

    # New path. ntime and npath are created consistently with the polygons
    # (different of mpath and mtime).
    npath = []
    ntime = []
    # curvature parameter
    etheta = []

    while i is not None:
        print i
        # new polygon
        pol1 = mpath[i:j + 1]
        # new path and new time. Dont include j+1 because it is the first element of the next polygon.
        npath = mpath[i:j].tolist() + npath
        ntime = mtime[i:j].tolist() + ntime

        # curvature parameter    
        # end point is true only in the first case, the polygon cover the hole cycle.
        final_point = len(pol1) if not etheta else len(pol1) - 1  # 0 if first time,else -1
        # print i, final_point

        etheta = compute_theta(pol1)[:final_point] + etheta

        polygons.append(pol1)

        j = i + 1
        i = polyline_closes_perpend(mpath[:j])
        if i is None:
            break
        intersection, newt = new_data_path(i, mpath[:j], mtime)
        mtime, mpath = insert_in_datapath(i, intersection, newt, mtime, mpath)

    return np.array(ntime), np.array(npath), polygons, etheta


def compute_theta(pol1):
    # np.linspace(0,1,len(pol1) - final_point, endpoint=ep).tolist()
    l = LineString(pol1)
    length = l.length

    # last point
    lp = pol1[0]

    etheta = []
    d = 0

    for i, p in enumerate(pol1):
        # Distance of the line segment
        d += LineString([lp, p]).length
        lp = p

        etheta.append(d / length)

    return etheta
