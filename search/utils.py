#
# A set of utilities to generate basic roads in CommonRoad and scenarios over them
#
from scipy.interpolate import splev, splprep
from scipy.spatial import geometric_slerp

from shapely import LineString, Point
from shapely.affinity import translate, rotate, scale

from numpy import repeat, array, sqrt, inf, cross, dot
from numpy import linspace, array, cross, dot

from numpy.ma import arange

from math import sin, cos, radians, degrees, atan2, copysign


def remove_duplicates(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]


def interpolate_and_resample(road_nodes, sampling_unit_m=1, rounding_precision = 3, interpolation_distance = 1, smoothness = 0, min_num_nodes = 20):
    """
    Interpolate the road points using cubic splines and ensure we handle 4F tuples for compatibility
    """
    old_x_vals = [t[0] for t in road_nodes]
    old_y_vals = [t[1] for t in road_nodes]

    # This is an approximation based on whatever input is given
    road_length = LineString([(t[0], t[1]) for t in road_nodes]).length

    num_nodes = int(road_length / sampling_unit_m)
    if num_nodes < min_num_nodes:
        num_nodes = min_num_nodes

    assert len(old_x_vals) >= 2, "You need at leas two road points to define a road"
    assert len(old_y_vals) >= 2, "You need at leas two road points to define a road"

    if len(old_x_vals) == 2:
        # With two points the only option is a straight segment
        k = 1
    elif len(old_x_vals) == 3:
        # With three points we use an arc, using linear interpolation will result in invalid road tests
        k = 2
    else:
        # Otherwise, use cubic splines
        k = 3

    pos_tck, pos_u = splprep([old_x_vals, old_y_vals], s=smoothness, k=k)
    step_size = 1 / num_nodes
    unew = arange(0, 1 + step_size, step_size)
    new_x_vals, new_y_vals = splev(unew, pos_tck)
    new_z_vals = repeat(0.0, len(unew))

    if len(road_nodes[0]) > 2:
        # Recompute width
        old_width_vals = [t[3] for t in road_nodes]
        width_tck, width_u = splprep([pos_u, old_width_vals], s=smoothness, k=k)
        _, new_width_vals = splev(unew, width_tck)

        # Return the 4-tuple with default z and default road width
        return list(
            zip(
                [round(v, rounding_precision) for v in new_x_vals],
                [round(v, rounding_precision) for v in new_y_vals],
                [round(v, rounding_precision) for v in new_z_vals],
                [round(v, rounding_precision) for v in new_width_vals],
            )
        )
    else:
        return list(
            zip(
                [round(v, rounding_precision) for v in new_x_vals],
                [round(v, rounding_precision) for v in new_y_vals],
            )
        )


def generate_road_points(initial_location_coords, initial_rotation, road_segment_dicts,
                         sampling_unit_m : float = 2.0, interpolate_turns_with_pts: int = 8):
    """
    Given a list of segments descirptions and an initial state computes the resulting road_central_points and the segment_boundaries
    """
    initial_location = Point(initial_location_coords)

    last_location = initial_location
    last_rotation = initial_rotation

    segment_boundaries = []
    segment_boundaries.append(last_location)

    road_central_points = [initial_location]
    len_coor = []

    for s in road_segment_dicts:
        # Generate the segment from the initial position and rotation
        # Then update the initial position and rotation for the next segment
        segment = None
        if s["type"] == "straight":
            # Create an horizontal line of given length from the origin
            segment = LineString([(x, 0) for x in linspace(0, s["length"], 8)])
            # Rotate it
            segment = rotate(segment, last_rotation, (0, 0))
            # Move it
            segment = translate(segment, last_location.x, last_location.y)
            # Update last rotation and last location
            last_rotation = (
                last_rotation  # Straight segments do not change the rotation
            )
            last_location = Point(list(segment.coords)[-1])

        elif s["type"] == "turn":
            # Generate the points over the circle with 1.0 radius
            # # Vector (0,1)
            # start = array([cos(radians(90.0)), sin(radians(90.0))])
            # # Compute this using the angle
            # end = array([cos(radians(90.0 - s["angle"])), sin(radians(90.0 - s["angle"]))])
            start = array([1, 0])

            # Make sure that positive is
            # TODO Pay attention to left/right positive/negative
            end = array([cos(radians(s["angle"])), sin(radians(s["angle"]))])
            # Interpolate over 8 points
            t_vals = linspace(0, 1, interpolate_turns_with_pts)
            result = geometric_slerp(start, end, t_vals)
            segment = LineString([Point(p[0], p[1]) for p in result])

            # Translate that back to origin
            segment = translate(segment, -1.0, 0.0)
            # Rotate
            if s["angle"] > 0:
                segment = rotate(segment, -90.0, (0.0, 0.0), use_radians=False)
            else:
                segment = rotate(segment, +90.0, (0.0, 0.0), use_radians=False)

            # Scale to radius on both x and y
            segment = scale(segment, s["radius"], s["radius"], 1.0, (0.0, 0.0))
            # Rotate it
            segment = rotate(segment, last_rotation, (0, 0))
            # Translate it
            segment = translate(segment, last_location.x, last_location.y)
            # Update last rotation and last location
            last_rotation = (
                last_rotation + s["angle"]
            )  # Straight segments do not change the rotation
            last_location = Point(list(segment.coords)[-1])

        if segment is not None:
            len_coor.append(len(list(segment.coords)))
            road_central_points.extend([Point(x, y) for x, y in list(segment.coords)])

            if last_location != initial_location:
                segment_boundaries.append(last_location)

    the_trajectory = LineString(
        remove_duplicates([(p.x, p.y) for p in road_central_points])
    )
    
    # Make sure we use as reference the NORTH
    the_trajectory = translate(the_trajectory, -initial_location.x, -initial_location.y)
    # Rotate by -90 deg
    the_trajectory = rotate(the_trajectory, +90.0, (0, 0))
    # Translate it back
    the_trajectory = translate(the_trajectory, +initial_location.x, +initial_location.y)

    ## TODO, do we really need those?
    segment_boundaries = LineString([(p.x,p.y) for p in segment_boundaries])
    # Make sure we use as reference the NORTH
    segment_boundaries = translate(segment_boundaries, -initial_location.x, -initial_location.y)
    # Rotate by -90 deg
    segment_boundaries = rotate(segment_boundaries, +90.0, (0, 0))
    # Translate it back
    segment_boundaries = translate(segment_boundaries, +initial_location.x, +initial_location.y)

    segment_boundaries = list(segment_boundaries.coords)

    # Interpolate and resample uniformly - Make sure no duplicates are there. Hopefully we do not change the order
    # TODO Sampling unit is 5 meters for the moment. Can be changed later
    interpolated_points = interpolate_and_resample(
        [(p[0], p[1]) for p in list(the_trajectory.coords)], sampling_unit_m=sampling_unit_m
    )

    # Concat the speed to the point
    road_central_points = list(the_trajectory.coords)
    start = 0
    sls = []
    sl_coor = []
    for s in len_coor:
        sl_coor.append([start, start + s])
        start = sl_coor[-1][1] - 1
    for s in sl_coor:
        sls.append(LineString(road_central_points[s[0] : s[1]]))

    road_central_points = []
    for line in sls:
        for p in interpolated_points:
            point = Point(p[0], p[1])
            if point.distance(line) < 0.5 and p not in road_central_points:
                road_central_points.append((p[0], p[1]))

    return road_central_points, segment_boundaries
