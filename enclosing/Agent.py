import math
from shapely.geometry import Point, Polygon
import numpy as np

from enclosing.path_joiner import perpendicular_line


class Agent:
    def __init__(self, (x1, y1), (x0, y0)):
        # Location
        self.x, self.y = x1, y1
        # Location in the curve
        self.s = None

        # Trajectory
        self.traj_x = [x1, x0]
        self.traj_y = [y1, y0]
        self.traj_t = []
        self.traj_s = []

    def move_on_boundary(self, boundary, vel):
        pr = Point((self.x, self.y))  # point in robot location
        circle = pr.buffer(vel).boundary  # circle around robot location

        boundary_polygon = Polygon(boundary).boundary

        intersection = circle.intersection(boundary_polygon)

        if intersection.is_empty:
            raise ValueError('Robot slower than boundary. Robot=(%f, %f)' % (self.x, self.y))

        ## select from intersected points
        p1, p2 = np.array(intersection[0].xy), np.array(intersection[1].xy)

        # import matplotlib.pylab as plt
        # print p1
        # plt.plot(p1[0], p1[1], 'x')
        # plt.plot(p2[0], p2[1], 'x')

        # distance to the second last point
        a1 = math.atan2((p1[1] - self.traj_y[-1]), (p1[0] - self.traj_x[-1]))
        a2 = math.atan2((p2[1] - self.traj_y[-1]), (p2[0] - self.traj_x[-1]))

        # path angle
        a = math.atan2(self.traj_y[-1] - self.traj_y[-2],
                       self.traj_x[-1] - self.traj_x[-2])

        ## convert to positive angles
        d1 = abs(a % (2 * math.pi) - a1)
        d2 = abs(a % (2 * math.pi) - a2)

        d1 = d1 if d1 < math.pi else abs(d1 - 2 * math.pi)
        d2 = d2 if d2 < math.pi else abs(d2 - 2 * math.pi)

        r = intersection[0] if d1 < d2 else intersection[1]
        nx, ny = r.xy

        self.x = nx[0]
        self.y = ny[0]

        self.traj_x.append(nx[0])
        self.traj_y.append(ny[0])
