import math
from shapely.geometry import Point, Polygon
import numpy as np

from enclosing.path_joiner import perpendicular_line


class Agent:
    def __init__(self, loc, theta=0.):
        (x1, y1) = loc
        # Location
        self.x, self.y = x1, y1
        # orientation
        self.theta = theta
        # Location in the curve
        self.s = None

        # Trajectory
        self.traj_x = [x1]
        self.traj_y = [y1]
        self.traj_t = []
        self.traj_s = []

    def move_on_boundary(self, boundary, vel):
        pr = Point((self.x, self.y))  # point in robot location
        circle = pr.buffer(vel).boundary  # circle around robot location

        boundary_polygon = Polygon(boundary).boundary

        intersection = circle.intersection(boundary_polygon)

        if intersection.is_empty:
            raise ValueError('Robot slower than boundary. Robot=(%f, %f)' % (self.x, self.y))

        # Intersecting points
        ip_x = np.array([p.xy[0][0] for p in intersection])
        ip_y = np.array([p.xy[1][0] for p in intersection])

        # angle with the robot
        angs = np.arctan2(ip_y - self.y, ip_x - self.x)
        # Convert to positive angles
        d = np.abs(self.theta - angs)
        d[d > math.pi] = np.abs(d[d > math.pi] - 2 * math.pi)

        #min_index = d.index(min(d))
        min_index = np.argmin(d)

        ## select from intersected points
        # p1, p2 = np.array(intersection[0].xy), np.array(intersection[1].xy)
        # distance to the second last point
        # a1 = math.atan2((p1[1] - self.y), (p1[0] - self.x))
        # a2 = math.atan2((p2[1] - self.y), (p2[0] - self.x))
        ## convert to positive angles
        # d1 = abs(self.theta - a1)
        # d2 = abs(self.theta - a2)
        # d1 = d1 if d1 < math.pi else abs(d1 - 2 * math.pi)
        # d2 = d2 if d2 < math.pi else abs(d2 - 2 * math.pi)
        # r = intersection[0] if d1 < d2 else intersection[1]
        # nx, ny = r.xy
        # self.x = nx[0]
        # self.y = ny[0]

        nx, ny = ip_x[min_index], ip_y[min_index]

        self.x = nx
        self.y = ny

        self.traj_x.append(nx)
        self.traj_y.append(ny)

        # Orientation between 0 and 2pi
        self.theta = math.atan2(self.traj_y[-1] - self.traj_y[-2],
                                self.traj_x[-1] - self.traj_x[-2]) % (2 * math.pi)
