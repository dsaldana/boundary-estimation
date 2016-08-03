import numpy as np
import math
import random
from math import atan2, sin, cos, pi
from shapely.geometry import Polygon

import matplotlib.pyplot as plt

### Generate
from enclosing.Agent import Agent
from enclosing.path_joiner import side_of_line
from enclosing.s_estimator import cut_polyline, get_perpendicular, update_zero, parametrize_polyset


def boundaries_on_time(t_steps=500, vel=.01):
    """
    Boundary on time
    :return: poligon for the boundary at each time step.
    """
    time = range(t_steps)
    nth = 200  # Number of partitions for theta
    lin_theta = np.linspace(0, 2 * math.pi, nth)

    boundaries = []

    for t in time:
        x = np.cos(lin_theta) + vel * math.cos( .00*t) *2000 # + vel * t
        y = np.sin(lin_theta)  # + vel * t
        boundaries.append(np.vstack((x, y)).T)

    return np.array(boundaries)


def init_agents(robot_locations, boundary0, boundary1):
    agents = [Agent(boundary1[il - 1], boundary0[il]) for il in robot_locations]
    return agents


def move_agents(agents1, boundaries1, from_t=2, to_t=80, vel=.1):
    """
    move the robots with constant velocity
    :param boundaries1:
    :param agents1:
    :param from_t:
    :param to_t:
    :param vel:
    """
    for boundary in boundaries1[from_t:to_t]:
        for a in agents1:
            a.move_on_boundary(boundary, vel)


def draw_initial_path(agents1, boundaries1, draw_paths=False):
    # Draw initial path
    if draw_paths:
        for a in agents1:
            plt.plot(a.traj_x, a.traj_y, '.-')
            plt.plot(a.traj_x[-1], a.traj_y[-1], 'o')

        bx, by = boundaries1[len(a.traj_x) - 1].T
        plt.plot(bx, by, '--')
        # plt.show()


######## Initial multi-line-string ################
def polylines_to_pieceswise_boundary(agents, draw_perps=False, draw_init_polyset=False):
    # polyline for robot 0
    polyset = [(agents[0].traj_x, agents[0].traj_y)]
    N = len(agents)

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
    ztx, zty = polyset[0][0], polyset[0][1]  # path robot 0
    (zp1, zp2) = get_perpendicular(ztx, zty)  # perpendicular
    idz = (0, len(ztx) - 1)  # polyline, id point in polyline
    zero = ztx[idz[1]], zty[idz[1]]  # Zero point

    # draw
    if draw_init_polyset:
        for (polyx, polyy) in polyset:
            plt.plot(polyx, polyy, 'b.-')
            plt.plot(zero[0], zero[1], 'x')
            # Zero perpendicular
            plt.plot([zp1[0], zp2[0]], [zp1[1], zp2[1]], '-')
            plt.xlim([-1.5, 2])  # FIXME static
            plt.ylim([-1.5, 1.5])
            # plt.show()
    return polyset, idz, zero, (zp1, zp2)


def draw_arc_param(ss, polyset, cols=None):
    for (i, j), s in ss.items():
        lx, ly = polyset[i]
        px, py = lx[j], ly[j]

        if cols is None:
            plt.plot(px, py, 'o')
        else:
            plt.plot(px, py, cols[i])
        plt.annotate('%.2f' % s, xy=(px - .1, py))


def update_s_locations(agents, ss, polyset):
    """
    Update location of the agents in the curve
    :param agents:
    :param ss:
    """
    N = len(agents)
    for i in range(N):
        a = agents[i]
        ns = ss[(i, len(polyset[i][0]) - 1)]  # Normalized e

        # a.s = ns
        if a.s is None:
            a.s = ns
        else:
            # Difference in the circular
            ds = ns - a.s
            ds = atan2(sin(ds * 2 * pi), cos(ds * 2 * pi)) / (2 * pi)
            # Move the current one
            a.s += ds


def update_pieceswise_boundary(i, agent, id_zero, zero, zero_line, polyset, draw_polysets=False):
    # new dot in the trajectory
    # t_px, t_py = agent.traj_x[-1], agent.traj_y[-1]

    #### Add new part
    polyx, polyy = polyset[i]
    polyx.append(agent.x)
    polyy.append(agent.y)

    ### Point 2 should belong to this side
    polyset_zero_x, polyset_zero_y = polyset[id_zero[0]]
    p2 = polyset_zero_x[id_zero[1] - 1], polyset_zero_y[id_zero[1] - 1]
    p2_side = side_of_line(zero_line, p2)

    #### Remove old part
    # perpendicular line
    # Remove after the cut
    polyset[i - 1] = cut_polyline((agent.traj_x, agent.traj_y), polyset[i - 1])

    # Update zero perpendicular
    zero, id_zero, (zp1, zp2), polyset = update_zero(p2_side, zero, zero_line, polyset)

    if draw_polysets:
        for (polyx, polyy) in polyset:
            plt.plot(polyx, polyy, 'b.-')
            # Zero
            plt.plot(zero[0], zero[1], 'x')
            # Zero perpendicular
            plt.plot([zp1[0], zp2[0]], [zp1[1], zp2[1]], '-')
            plt.xlim([-1.5, 2])
            plt.ylim([-1.5, 1.5])
        plt.show()

    return polyset, id_zero, zero, (zp1, zp2)


def vel_control(i, agents, ke=.4, min_vel=0.05):
    N = len(agents)
    a = agents[i]

    ### Velocity control
    aa = agents[i - 1].s  # agent after
    ab = agents[(i + 1) % N].s  # agent before

    # s average
    aver = (aa + ab) / 2.
    #
    if i == N - 1:
        aver -= .5
    if i == 0:
        aver += .5

    e = (aver - a.s)
    vel = .2 + ke * e

    if vel < min_vel:
        vel = min_vel
    # print 'i=%d ab=%.2f aa=%.2f s=%.2f, av=%.2f vel=%.2f e=%f' % (i, ab, aa, a.s, aver, vel, e)

    return vel, e


def arc_lenght(boundary):
    pol = Polygon(boundary)
    return pol.length


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


def move_along_boundary(agents, initial_steps, boundaries, (id_zero, zero_point, zero_line, polyset), running_steps=140,
                        p_gain=.4):
    N = len(agents)
    errors = []
    polysets = []

    # Move the boundary
    for k in range(running_steps):
        boundary = boundaries[initial_steps + k]

        # For each agent
        single_error = []
        for i in range(N):
            a = agents[i]

            vel, e = vel_control(i, agents, ke=p_gain)
            a.move_on_boundary(boundary, vel)

            single_error.append(e)
            ### Update piecewise boundary
            polyset, id_zero, zero_point, zero_line = update_pieceswise_boundary(i, a, id_zero, zero_point, zero_line,
                                                                                 polyset,
                                                                                 draw_polysets=False)
            # Parametrize curve
            ss = parametrize_polyset(polyset, id_zero)

            # Update location of the agents in the curve
            update_s_locations(agents, ss, polyset)
            a.traj_s.append(a.s)
            a.traj_t.append(k)

        # polysets.append(copy(polyset))
        polysets.append([np.copy(p) for p in polyset])
        errors.append(single_error)

    return errors, polysets


def extract_dataset(agents):
    tt, th, xx, yy = [], [], [], []
    for i in range(len(agents[0].traj_s)):
        for a in agents:
            tt.append(i)
            th.append(a.traj_s[i])
            xx.append(a.traj_x[i])
            yy.append(a.traj_y[i])

    return tt, th, xx, yy
