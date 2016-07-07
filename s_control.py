from math import atan2, sin, cos, pi
import numpy as np
from enclosing.cooperative_enclosing import boundaries_on_time, init_agents, move_agents, draw_initial_path, \
    polylines_to_pieceswise_boundary, draw_arc_param, update_pieceswise_boundary, update_s_locations
import matplotlib.pyplot as plt

#######################################
# Compute boundaries
from enclosing.s_estimator import cut_polyline, update_zero, parametrize_polyset

boundaries = boundaries_on_time()

############ Initial conditions ###############
# Graphic Debug
draw_paths = False
draw_init_polyset = False
draw_polysets = False
draw_arc = False

# Number of robots
N = 3
# Compute boundaries
boundaries = boundaries_on_time()

## Creating agents
agents = init_agents(N, boundaries[0], boundaries[1])

######## Initial paths
initial_steps = 80
move_agents(agents, boundaries, from_t=2, to_t=initial_steps, vel=.1)
# Draw
draw_initial_path(agents, boundaries, draw_paths=draw_paths)

###### Init piecewise boundary
polyset, idz, zero_point, zero_line = polylines_to_pieceswise_boundary(agents, draw_perps=False,
                                                                       draw_init_polyset=draw_init_polyset)
ss = parametrize_polyset(polyset, idz)
if draw_arc:
    draw_arc_param(ss, polyset)
    plt.show()

# Location of the agent in the curve.
update_s_locations(agents, ss, polyset)

##############
# Update multi-line string
###############

errors = []

# Move the boundary
for k in range(140):
    print k
    boundary = boundaries[initial_steps + k]

    # For each agent
    single_error=[]
    for i in range(N):
        a = agents[i]

        ### Velocity control
        aa = agents[i - 1].s  # agent after
        ab = agents[(i + 1) % N].s  # agent before
        # if a.s > aa:
        #     aa += 1
        # if a.s < ab:
        #     ab -= 1
        # if aa < ab:
        #     aa += 1

        # s average
        aver = (aa + ab) / 2.
        #
        if i == N - 1:
            aver -= .5
        if i == 0:
            aver += .5

        # Closest distance to the average
        if abs(a.s - aver) > abs((1 + aver) - a.s):
            aver += 1

        k = .4
        vel = .1

        e = (aver - a.s)

        # if e > 1:
        #     e %= 1
        # e = atan2(sin(e*2*pi), cos(e*2*pi))/ (2*pi)


        vel = .2 + k * e

        if vel < 0.05:
            vel = 0.05

        a.move_on_boundary(boundary, vel)
        print 'i=%d ab=%.2f aa=%.2f s=%.2f, av=%.2f vel=%.2f e=%f' % (i, ab, aa, a.s, aver, vel, e)
        single_error.append(e)
        ### Update piecewise boundary
        polyset, id_zero, zero_point, zero_line = update_pieceswise_boundary(i, a, zero_point, zero_line, polyset,

                                                                             draw_polysets=draw_polysets)
        # Parametrize curve
        ss = parametrize_polyset(polyset, id_zero)

        # Update location of the agents in the curve
        update_s_locations(agents, ss, polyset)

    errors.append(single_error)
    if draw_arc:
        draw_arc_param(ss, polyset)
        plt.show()


plt.plot(np.array(errors))
plt.show()