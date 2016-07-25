import numpy as np

import matplotlib.pyplot as plt


from enclosing.cooperative_enclosing import boundaries_on_time, init_agents, move_agents, draw_initial_path, \
    polylines_to_pieceswise_boundary, draw_arc_param, update_pieceswise_boundary, update_s_locations, vel_control, \
    arc_lenght

from enclosing.s_estimator import parametrize_polyset

############ Initial conditions ###############
# Graphic Debug
draw_paths = False
draw_init_polyset = False
draw_polysets = False
draw_arc = True

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

### Move the boundary
for k in range(140):
    print k
    boundary = boundaries[initial_steps + k]

    # For each agent
    single_error = []
    for i in range(N):
        a = agents[i]

        vel, e = vel_control(i, agents, ke=.4)
        l = arc_lenght(boundary)

        dot_r = .1 * vel*l
        print vel, l, dot_r
        a.move_on_boundary(boundary, dot_r)

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
