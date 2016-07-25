import numpy as np

import matplotlib.pyplot as plt

from enclosing.cooperative_enclosing import boundaries_on_time, init_agents, move_agents, draw_initial_path, \
    polylines_to_pieceswise_boundary, draw_arc_param, update_pieceswise_boundary, update_s_locations, vel_control, \
    arc_lenght, move_along_boundary

from enclosing.s_estimator import parametrize_polyset

############ Initial conditions ###############
# Graphic Debug
draw_paths = False
draw_init_polyset = False
draw_polysets = True
draw_arc = False

# Number of robots
N = 3
# Compute boundaries
boundaries = boundaries_on_time()
# boundaries = boundaries_on_time(vel=.02)

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
# Move robots along the boundary
###############
errors, polysets = move_along_boundary(agents, initial_steps, boundaries, (zero_point, zero_line, polyset),
                                       running_steps=140)

# Plot first polyset
for i in range(N):
    plt.plot(polysets[0][i][0], polysets[0][i][1])
# plt.plot(errors)
plt.show()
