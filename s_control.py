from enclosing.cooperative_enclosing import boundaries_on_time, init_agents, move_agents, draw_initial_path, \
    polylines_to_pieceswise_boundary, draw_arc_param2, update_pieceswise_boundary
import matplotlib.pyplot as plt

#######################################
# Compute boundaries
from enclosing.s_estimator import cut_polyline, update_zero, parametrize_polyset
from enclosing.testpoly import draw_arc_param

boundaries = boundaries_on_time()

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
draw_arc_param(polyset, idz, draw_arc=draw_arc)

##############
# Update multi-line string
###############

# Move the boundary
for k in range(50):
    boundary = boundaries[initial_steps + k]

    # For each agent
    for i in range(N):
        a = agents[i]

        vel = .1
        a.move_on_boundary(boundary, vel)

        polyset, id_zero, zero_point, zero_line = update_pieceswise_boundary(i, a, zero_point, zero_line, polyset,
                                                                             draw_polysets=draw_polysets)



    if draw_arc:
        ss = parametrize_polyset(polyset, id_zero)
        draw_arc_param(ss, polyset, draw_arc=draw_arc)
        plt.show()

