from shapely.geometry import LineString, MultiLineString, Point

from enclosing.path_joiner import perpendicular_line, side_of_line, euclidean_distance
import matplotlib.pyplot as plt


def identify_cut((tx, ty), polyline):
    """

    :param polyline:
    :return:
    """
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
    # for j in range(len(oldp_x) - 1, 0, -1):
    for j in range(len(oldp_x) - 2, 0, -1):  # XXX: the line above is the best.
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
    #### Using perpendicular line
    # j = identify_cut((tx, ty), polyline)
    # if j is None:
    #     raise ValueError('Error: no intersection between path and polyline')
    #
    # # Point of the intersection
    # pline = get_perpendicular(tx, ty)
    # new_p = compute_intersection(j, polyline, pline)

    #### using closest point
    p = Point(tx[-1], ty[-1])
    ls = LineString([(xi, yi) for xi, yi in zip(polyline[0], polyline[1])])
    dp = ls.project(p)
    closest_point = ls.interpolate(dp)
    nx, ny = closest_point.xy
    new_p = nx[0], ny[0]

    oldp_x, oldp_y = polyline
    for j in range(len(oldp_x) - 2, 0, -1):
        p1 = oldp_x[j], oldp_y[j]
        p2 = oldp_x[j - 1], oldp_y[j - 1]
        p1, p2 = Point(p1), Point(p2)

        if ls.project(p2) <= dp <= ls.project(p1):
            break
    else:
        raise ValueError('Error: no intersection with closest point')

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


def identify_intersection(p2_side_old, perp_line, polyline):
    # Last linestring from robot im1
    oldp_x, oldp_y = polyline

    # Current zero
    # polyline[]


    # Moving backwards trough the last polyline
    for j in range(len(oldp_x) - 1, 0, -1):
        # for j in range(len(oldp_x)-1):
        p1 = oldp_x[j], oldp_y[j]
        p2 = oldp_x[j - 1], oldp_y[j - 1]

        ### side of p1
        p1_side = side_of_line(perp_line, p1)
        p2_side = side_of_line(perp_line, p2)

        # if the points are in different sides
        if p1_side != p2_side and p2_side == p2_side_old:
            return j
    return None


def update_zero(p2_side, zero, perp_line, polyset):
    # Identify the intersection between the perpendicular line and the polyset part
    min_d = 10000000  # Minimum distance
    id_c = None
    c = None  # Candiate point
    for i, polyline in enumerate(polyset):
        j = identify_intersection(p2_side, perp_line, polyline)
        if j is not None:
            new_p = compute_intersection(j, polyline, perp_line)

            d = euclidean_distance(new_p, zero)

            if d < min_d:
                min_d = d
                id_c = [i, j]
                c = new_p

    # if the intersection point already exists
    i, j = id_c
    polyline = polyset[i]
    if euclidean_distance(c, (polyline[0][j], polyline[1][j])) == 0:
        perp_line = get_perpendicular(polyline[0][:j + 1], polyline[1][:j + 1])
        return c, id_c, perp_line, polyset
    elif euclidean_distance(c, (polyline[0][j - 1], polyline[1][j - 1])) == 0:
        id_c[1] -= 1
        perp_line = get_perpendicular(polyline[0][:j], polyline[1][:j])
        return c, id_c, perp_line, polyset

    # add the point to the polyline
    tx, ty = polyset[i]
    polyline = tx[:j] + [c[0]] + tx[j:], ty[:j] + [c[1]] + ty[j:]
    perp_line = get_perpendicular(polyline[0][:j + 1], polyline[1][:j + 1])
    id_c[1] += 1
    polyset[i] = polyline
    return c, id_c, perp_line, polyset


def parametrize_polyset(polyset, id_zero):
    """
    Parametrize the polyset by arc-lenght
    :param polyset:
    :param id_zero:
    """
    line_ponts = [[(x, y) for x, y in zip(linex, liney)] for (linex, liney) in polyset]
    # Arc length
    mls = MultiLineString(line_ponts)

    arc_len = mls.length

    # Number of polylines
    N = len(polyset)

    # Zero (iz is robot, jz is the id of the point in the linestring of robot iz)
    iz, jz = id_zero
    j = None
    s_param = {}
    accum = 1

    # For each polyline, starting by the zero
    for p in range(N):
        i = (iz + p) % N
        line = line_ponts[i]

        if j is None:
            j = jz
        else:
            j = len(line) - 1

        # First point
        s_param[(i, j)] = accum
        # Following points
        while j > 0:
            d = euclidean_distance(line[j], line[j - 1])
            accum -= d / arc_len
            s_param[(i, j - 1)] = accum
            j -= 1

    # Remaining points
    line = line_ponts[iz]

    accum = 0
    for j in range(jz + 1, len(line)):
        # print jz, j
        d = euclidean_distance(line[j], line[j - 1])
        accum += d / arc_len
        s_param[(iz, j)] = accum

    return s_param
