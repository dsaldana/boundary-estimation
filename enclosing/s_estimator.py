from shapely.geometry import LineString

from enclosing.path_joiner import perpendicular_line, side_of_line
import matplotlib.pyplot as plt


def identify_cut((tx, ty), polyline):
    # normal vector for robot i
    p1, p2 = perpendicular_line((tx[-1], ty[-1]), (tx[-2], ty[-2]), ddd=1000000)
    perp_line = [p1, p2]

    per_seg = LineString([p1, p2])

    # Last linestring from robot im1
    oldp_x, oldp_y = polyline
    # ls_rim1 = LineString(oldp_x, oldp_y)

    # Side of the second point
    side = side_of_line([p1, p2], (tx[-2], ty[-2]))

    # Moving backwards trough the last polyline
    for j in range(len(oldp_x) - 1, 2, -1):
        # for j in range(len(oldp_x)-1):
        p1 = oldp_x[j], oldp_y[j]
        p2 = oldp_x[j - 1], oldp_y[j - 1]

        # plt.plot([oldp_x[j], oldp_x[j - 1]], [oldp_y[j], oldp_y[j - 1]])
        # plt.draw()
        # plt.pause(.1)


        ### side of p1
        p1_side = side_of_line(perp_line, p1)
        # p1 in the wrong side
        if p1_side == side:
            # print "-",i, p1_side
            continue
        # side of p2
        p2_side = side_of_line(perp_line, p2)
        if p2_side == side or p2_side == 0:
            return j
    return None
