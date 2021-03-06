import numpy as np

import matplotlib.pyplot as plt

from enclosing.cooperative_enclosing import boundaries_on_time, init_agents, move_agents, draw_initial_path, \
    polylines_to_pieceswise_boundary, draw_arc_param, update_pieceswise_boundary, update_s_locations, vel_control, \
    arc_lenght, move_along_boundary, extract_dataset

from enclosing.s_estimator import parametrize_polyset

############ Initial conditions ###############
# Graphic Debug
draw_paths = False
draw_init_polyset = False
draw_polysets = True
draw_arc = False

# Compute boundaries
# boundaries = boundaries_on_time(vel=.01)
# robot_speed = .2

boundaries = np.load('boundaries.npy')
robot_speed = 6.


## Creating agents
# Initial locations
M = len(boundaries[0])
iloc = [int(.4 * M), int(.2 * M), int(.1 * M)]
# iloc = [int(.4 * M)]
# iloc = [int(.4 * M), int(.8 * M)]
agents = init_agents(iloc, boundaries[0])

######## Initial paths
initial_steps = 80
initial_steps = 50
move_agents(agents, boundaries, from_t=1, to_t=initial_steps, vel=robot_speed)

# Draw
draw_initial_path(agents, boundaries, draw_paths=draw_paths)
plt.show()

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
errors, polysets = move_along_boundary(agents, initial_steps, boundaries, (idz, zero_point, zero_line, polyset),
                                       running_steps=400, robot_speed=2*robot_speed)

#############
### Get DATA
############
# Remove the two initial steps
for a in agents:
    a.traj_x = a.traj_x[2:]
    a.traj_y = a.traj_y[2:]

## Dataset
dataset = extract_dataset(agents)
# print dataset[1]

# print np.array(dataset).shape
# Plot first polyset
# for i in range(N):
#     plt.plot(polysets[0][i][0], polysets[0][i][1])
# plt.plot(errors)
plt.show()
