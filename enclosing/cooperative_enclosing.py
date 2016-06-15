import numpy as np
import math
import random

import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon, Point

### Generate
from enclosing.path_joiner import perpendicular_line, side_of_line
from enclosing.s_estimator import identify_cut, compute_intersection


def boundaries_on_time(t_steps=500):
    """
    Boundary on time
    :return: poligon for the boundary at each time step.
    """
    time = range(t_steps)
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
# iloc = np.random.randint(0, len(boundary0), N)
M = len(boundary0)
iloc = [7 * M / 10, M / 2, M / 10]
agents = [Agent(boundary1[i - 1], boundary0[i]) for i in iloc]

# bx, by = boundary1.T
# plt.plot(bx, by)

####### Initial path
initial_steps = 80
# move the robots with constant velocity
for boundary in boundaries[2:initial_steps]:
    for a in agents:
        # vel = .3 * random.random() + .1
        a.move_on_boundary(boundary, vel)

# Draw initial path
for a in agents:
    plt.plot(a.traj_x[-1], a.traj_y[-1], 'o')
    plt.plot(a.traj_x, a.traj_y, '.')

bx, by = boundaries[initial_steps - 1].T
plt.plot(bx, by, '--')
# plt.show()
# plt.ion()

#### Joining paths
# polyline for robot 0
polyset = [(agents[0].traj_x, agents[0].traj_y)]

for i in range(1, N + 1):
    a = agents[i % N]
    # Trajectory for robot i
    tx, ty = a.traj_x, a.traj_y
    # Last points of the trajectory
    lp1 = (tx[-1], ty[-1])
    lp2 = (tx[-2], ty[-2])
    # draw debug
    p1, p2 = perpendicular_line(lp1, lp2, ddd=1000000)
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], '-')
    plt.xlim([-1.5, 2])
    plt.ylim([-1.5, 1.5])
    # plt.draw()
    # plt.pause(5.5)

    # point of the cut
    (polyx, polyy) = polyset[i - 1]
    j = identify_cut((tx, ty), (polyx, polyy))
    # Point of the intersection
    new_p = compute_intersection(j, (polyx, polyy), (p1, p2))

    #plt.plot(new_p[0], new_p[1], 'v')


    # Remove after the cut
    polyset[i - 1] = [[new_p[0]] + polyx[j:], [new_p[1]] + polyy[j:]]

    # add polyline (the first line was added at the beginning)
    if not i == N:
        polyset.append([tx, ty])

    # plt.show()
    print j

for (polyx, polyy) in polyset:
    plt.plot(polyx, polyy, 'b')

plt.show()
# for a in agents:
#     plt.plot(a.traj_x, a.traj_y)
#
#
# tail = 40
# plt.ion()
#
# for i in range(tail + 1, len(boundaries)):
#     plt.clf()
#     bx, by = boundaries[i].T
#     plt.plot(bx, by)
#
#     for a in agents:
#         plt.plot(a.traj_x[i - tail:i + 1], a.traj_y[i - tail:i + 1], '.-')
#         plt.plot(a.traj_x[i], a.traj_y[i], 'o')
#     plt.draw()
#     plt.pause(0.5)
#
