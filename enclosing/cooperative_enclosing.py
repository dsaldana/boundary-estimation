import numpy as np
import math
import random

import matplotlib.pyplot as plt

### Generate
from enclosing.Agent import Agent
from enclosing.s_estimator import cut_polyline, get_perpendicular, update_zero, parametrize_polyset


def boundaries_on_time(t_steps=500):
    """
    Boundary on time
    :return: poligon for the boundary at each time step.
    """
    time = range(t_steps)
    nth = 200  # Number of partitions for theta
    lin_theta = np.linspace(0, 2 * math.pi, nth)

    boundaries = []

    for t in time:
        x = np.cos(lin_theta) + math.sin(.01 * t)
        y = np.sin(lin_theta)
        boundaries.append(np.vstack((x, y)).T)

    return np.array(boundaries)


# Draw
draw_perps = False  # Perpendicular lines
draw_paths = False
draw_init_polyset = False
draw_polysets = True
draw_arc = True

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

####### Initial path ############
initial_steps = 80
# move the robots with constant velocity
for boundary in boundaries[2:initial_steps]:
    for a in agents:
        # vel = .3 * random.random() + .1
        a.move_on_boundary(boundary, vel)

# Draw initial path
if draw_paths:
    for a in agents:
        plt.plot(a.traj_x[-1], a.traj_y[-1], 'o')
        plt.plot(a.traj_x, a.traj_y, '-')

    bx, by = boundaries[initial_steps - 1].T
    plt.plot(bx, by, '--')

######## Initial multi-line-string ################
# polyline for robot 0
polyset = [(agents[0].traj_x, agents[0].traj_y)]

# For each agent
for i in range(1, N + 1):
    a = agents[i % N]
    # Trajectory for robot i
    tx, ty = a.traj_x, a.traj_y

    # Remove after the cut
    polyset[i - 1] = cut_polyline((tx, ty), polyset[i - 1])

    # add polyline (the first line was added at the beginning)
    if not i == N:
        polyset.append([tx, ty])

    # Remove path
    a.traj_x, a.traj_y = a.traj_x[-2:], a.traj_y[-2:]
    # plt.show()

    ### Draw
    if draw_perps:
        (p1, p2) = get_perpendicular(tx, ty)
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], '-')
        plt.xlim([-1.5, 2])
        plt.ylim([-1.5, 1.5])

## Zero point
ztx, zty = polyset[0][0], polyset[0][1]
idz = (0, len(ztx) - 1)  # polyline, id point in polyline
zero = ztx[idz[1]], zty[idz[1]]
(zp1, zp2) = get_perpendicular(ztx, zty)

# draw
if draw_init_polyset:
    for (polyx, polyy) in polyset:
        plt.plot(polyx, polyy, 'b.-')
        plt.plot(zero[0], zero[1], 'x')
        # Zero perpendicular
        plt.plot([zp1[0], zp2[0]], [zp1[1], zp2[1]], '-')
        plt.xlim([-1.5, 2])
        plt.ylim([-1.5, 1.5])

if draw_arc:
    ss = parametrize_polyset(polyset, idz)

    def draw_arc_param(ss, polyset):
        for (i, j), s in ss.items():
            lx, ly = polyset[i]
            px, py = lx[j], ly[j]

            plt.plot(px, py, 'o')
            plt.annotate('%.4f' % s, xy=(px + .0, py))
    draw_arc_param(ss, polyset)

plt.show()

###############
# Update multi-line string
###############






# Move the boundary
for k in range(50):
    boundary = boundaries[initial_steps + k]

    # For each agent
    for i in range(N):
        a = agents[i]
        a.move_on_boundary(boundary, .1)

        # new dot in the trajectory
        t_px, t_py = a.traj_x[-1], a.traj_y[-1]

        #### Add new part
        polyx, polyy = polyset[i]
        polyx.append(a.x)
        polyy.append(a.y)

        #### Remove old part
        # perpendicular line
        # Remove after the cut
        polyset[i - 1] = cut_polyline((a.traj_x, a.traj_y), polyset[i - 1])

        # Update zero perpendicular
        zero, id_zero, (zp1, zp2), polyset = update_zero(zero, (zp1, zp2), polyset)

        # Arc-length

    if draw_polysets:
        for (polyx, polyy) in polyset:
            plt.plot(polyx, polyy, 'b.-')
            # Zero
            plt.plot(zero[0], zero[1], 'x')
            # Zero perpendicular
            plt.plot([zp1[0], zp2[0]], [zp1[1], zp2[1]], '-')
            plt.xlim([-1.5, 2])
            plt.ylim([-1.5, 1.5])


    if draw_arc:
        ss = parametrize_polyset(polyset, id_zero)
        draw_arc_param(ss, polyset)



    plt.show()







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
