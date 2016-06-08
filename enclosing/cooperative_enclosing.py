import numpy as np
import math
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

        a1 = math.atan2((p1[1] - self.traj_y[-1]), (p1[0] - self.traj_x[-1]))
        a2 = math.atan2((p2[1] - self.traj_y[1]), (p2[0] - self.traj_x[-1]))

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

        self.traj_x.append(nx)
        self.traj_y.append(ny)


#######################################
# Compute boundaries
boundaries = boundaries_on_time()

############ Initial conditions ###############
# Number of robots
N = 3
vel = .3

## Creating agents
boundary0, boundary1 = boundaries[0], boundaries[1]
iloc = np.random.randint(0, len(boundary0), N)
agents = [Agent(boundary1[i - 1], boundary0[i]) for i in iloc]



bx, by = boundary1.T
plt.plot(bx, by)

for a in agents:
    plt.plot(a.x, a.y, 'o')

    a.move_on_boundary(boundaries[2], vel)

    plt.plot(a.x, a.y, 'v')

plt.show()
