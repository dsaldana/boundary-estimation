from shapely.geometry import LineString

from enclosing.path_joiner import perpendicular_line, side_of_line
import matplotlib.pyplot as plt


def identify_cut((tx, ty), polyline):
    # normal vector for robot i
    perp_line = get_perpendicular(tx, ty)

    # plt.clf()
    # plt.plot([perp_line[0][0], perp_line[1][0]], [perp_line[0][1], perp_line[1][1]])
    # plt.plot(polyline[0], polyline[1], 'o')
    # plt.show()


    # Last linestring from robot im1
    oldp_x, oldp_y = polyline

    # Side of the second point
    side = side_of_line(perp_line, (tx[-2], ty[-2]))

    # Moving backwards trough the last polyline
    for j in range(len(oldp_x) - 1, 0, -1):
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

def get_perpendicular(tx, ty, delta=100000000):
    # Last points of the trajectory
    lp1 = (tx[-1], ty[-1])
    lp2 = (tx[-2], ty[-2])
    # draw debug
    p1, p2 = perpendicular_line(lp1, lp2, ddd=delta)

    return p1, p2

def cut_polyline((tx, ty), polyline):



    j = identify_cut((tx, ty), polyline)
    if j is None:
        print 'Error: no intersection between path and polyline'
        return None

    # Point of the intersection
    pline = get_perpendicular(tx, ty)
    new_p = compute_intersection(j, polyline, pline)
    # New polyline after cut
    new_poly = [[new_p[0]] + polyline[0][j:], [new_p[1]] + polyline[1][j:]]
    return new_poly


def compute_intersection(i, (polyx, polyy), perp_line):
    """
    new point in the intersection.
    It creates a perpendicular line from the i-point in path.
    and returns the intersection between the perpendicular and the path.
    """
    ########## Intersection point #########
    # segment of interest.
    seg = LineString([(polyx[i], polyy[i]),
                      (polyx[i - 1], polyy[i - 1])])
    # perpendicular segment
    # p1, p2 = perpendicular_line(lp1, lp2, ddd=100000)
    per_seg = LineString(perp_line)

    # plt.plot((polyx[i], polyx[i - 1]),
    #          (polyy[i], polyy[i - 1]), 'r')
    # plt.plot([p1[0], p2[0]],[p1[1], p2[1]], 'r')

    inter = per_seg.intersection(seg)

    intersection = None

    ## intersection
    if inter.type == 'Point':
        intersection = inter.coords[0]
    elif inter.type == 'MultiPoint':
        intersection = inter[0].coords[0]
    elif inter.type == 'GeometryCollection':
        print "---NO INTERSECTION "  # , [path[i], path[i - 1]], i
        # intersection = inter[0].coords[0]
    else:
        print "Unknown type", inter.type

    return intersection
