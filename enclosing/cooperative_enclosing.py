import numpy as np
import math
import random

import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon, Point


### Generate
def boundaries_on_time():
    """
    Boundary on time
    :return: poligon for the boundary at each time step.
    """
    time = range(500)
    nth = 200  # Number of partitions for theta
    lin_theta = np.linspace(0, 2 * math.pi, nth)

    # xx, yy = [], []
    boundaries = []

    for t in time:
        x = np.cos(lin_theta) + math.sin(.01 * t)
        y = np.sin(lin_theta)
        # xx.append(x)
        # yy.append(y)
        boundaries.append(np.vstack((x, y)).T)
    # xx = np.array(xx)
    # yy = np.array(yy)

    return np.array(boundaries)

    # return time, xx, yy


############################################
class Agent:
    def __init__(self, (x1, y1), (x0, y0)):
        # Location
        self.x, self.y = x1, y1

        # Trajectory
        self.traj_x = [x1, x0]
        self.traj_y = [y1, y0]

    def move_on_boundary(self, boundary, vel):
        pr = Point((self.x, self.y))  # point in robot location
        circle = pr.buffer(vel).boundary  # circle around robot location

        boundary_polygon = Polygon(boundary).boundary

        intersection = circle.intersection(boundary_polygon)

        ## select from intersected points
        p1, p2 = np.array(intersection[0].xy), np.array(intersection[1].xy)

        # distance to the second last point
        # d1 = math.atan2((p1[1] - self.traj_y[-2]), (p1[0] - self.traj_x[-2]))
        # d2 = math.atan2((p2[1] - self.traj_y[-2]), (p2[0] - self.traj_x[-2]))
        # r = p1 if d1 > d2 else p2
        # nx, ny = r
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


#######################################
# Compute boundaries
boundaries = boundaries_on_time()

############ Initial conditions ###############
# Number of robots
N = 3
vel = .1

## Creating agents
boundary0, boundary1 = boundaries[0], boundaries[1]
iloc = np.random.randint(0, len(boundary0), N)
agents = [Agent(boundary1[i - 1], boundary0[i]) for i in iloc]

# bx, by = boundary1.T
# plt.plot(bx, by)

# move the robots with constant velocity
for boundary in boundaries[2:]:
    for a in agents:
        # vel = .3 * random.random() + .1
        a.move_on_boundary(boundary, vel)

bx, by = boundaries[-1].T
plt.plot(bx, by)

# for a in agents:
#     plt.plot(a.traj_x, a.traj_y)


tail = 40
plt.ion()

for i in range(tail + 1, len(boundaries)):
    plt.clf()
    bx, by = boundaries[i].T
    plt.plot(bx, by)

    for a in agents:
        plt.plot(a.traj_x[i - tail:i + 1], a.traj_y[i - tail:i + 1], '.-')
        plt.plot(a.traj_x[i], a.traj_y[i], 'o')
    plt.draw()
    plt.pause(0.5)

plt.show()