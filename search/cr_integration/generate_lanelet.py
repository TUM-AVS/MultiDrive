import numpy as np

from typing import Tuple, Set

from shapely import (
    Point,
    LineString,
    MultiPoint,
    buffer,
    get_coordinates,
    offset_curve,
    centroid,
    convex_hull,
    union,
    union_all,
    difference
)
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad.common.common_lanelet import LaneletType



def create_centre_lanelet(
    left_lanelet: np.ndarray, right_lanelet: np.ndarray
) -> np.ndarray:
    """The function takes two arrays which constitute the left and right side of a road as vertices and returns the calculated centre vertex between them."""
    centre_vertex = list()
    for i in range(len(left_lanelet)):
        p = centroid(MultiPoint([(left_lanelet[i]), (right_lanelet[i])]))
        p_coord = get_coordinates(p)[0]
        centre_vertex.append(p_coord)
    for x, y in centre_vertex:
        x = round(x, 6)
        y = round(y, 6)
    centre_vertex = np.array(centre_vertex)
    # print('centre vertex ---',centre_vertex)
    return centre_vertex


def create_lanelet_vertices_offset_curve(
    original_vertex: np.ndarray,
    buffer_size: int,
    quad_segs: int = 0,
    join_style: str = "mitre",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The function takes a centre lane vertex and creates left and right vertices that constitute the left and right side of the lane based on buffer size."""
    centre_vertex = LineString(original_vertex)
    left_lane_vertex = offset_curve(
        centre_vertex, buffer_size, quad_segs=quad_segs, join_style=join_style
    )
    right_lane_vertex = offset_curve(
        centre_vertex, (-1 * buffer_size), quad_segs=quad_segs, join_style=join_style
    )

    left_lane_vertex = get_coordinates(left_lane_vertex)
    right_lane_vertex = get_coordinates(right_lane_vertex)

    new_centre_vertex = create_centre_lanelet(left_lane_vertex, right_lane_vertex)

    # print(f"top_lane_vertex: \n{left_lane_vertex}")
    # print(f"bottom_lane_vertex: \n{right_lane_vertex}")
    # print(f"new_centre_vertex: {new_centre_vertex}")
    # print(f"original centre vertex: {original_vertex}")

    return left_lane_vertex, right_lane_vertex, new_centre_vertex


def create_vertex_set(sequence: np.ndarray) -> Set[Tuple]:
    """The function creates a set of tuples based on an 2d array in order to remove duplicate coords in the array."""
    return set([tuple(x) for x in sequence])


def create_lane_vertex(side_lane: Set[Tuple], centre_lane: Set[Tuple]) -> np.ndarray:
    """The function aims to find the points in the side lane that are not included in centre lane. And converts from set of tuples to 2d numpy array."""
    side_lane_vertex = side_lane.difference(centre_lane)
    side_lane_vertex = np.array([list(x) for x in side_lane_vertex])
    # print(f'unsorted vertex: {side_lane_vertex}')
    # side_lane_vertex.sort(axis=0)
    side_lane_vertex = np.array(sorted(side_lane_vertex, key=lambda x: x[0]))
    return side_lane_vertex


def get_line_coords(main_lane, remove_lane):
    """ """
    
    main_lane_list = main_lane.tolist()
    remove_lane_list = remove_lane.tolist()

    return_lane = [ pos for pos in main_lane_list if pos not in remove_lane_list]
    # for pos in main_lane_list:
    #     if pos in return_lane or pos in remove_lane_list:
    #         continue
    #     else:
    #         return_lane.append(pos)

    def debug_plot():
        import matplotlib.pyplot as plt
        plt.plot([p[0] for p in main_lane_list], [p[1] for p in main_lane_list], ".", color="green")
        plt.plot([p[0] for p in remove_lane_list], [p[1] for p in remove_lane_list], ".", color="green")
        plt.plot([p[0] for p in return_lane], [p[1] for p in return_lane], "s", color="black", alpha=0.2)

        plt.gca().set_aspect("equal")
        plt.show()

    return np.array(return_lane)


def create_lanelet_vertices_buffer(
    original_vertex: np.ndarray,
    buffer_size: int,
    quad_segs: int = 0,
    cap_style: str = "square",
    join_style: str = "mitre",
    sampling_unit_m: float = 2.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Takes a 2d numpy array of cartesian coordinate tuples which is a single vertex. Creates a parallel buffer vertex on either side.
    """

    centre_vertex = LineString(original_vertex)

    left_lane = buffer(
        centre_vertex,
        buffer_size,
        quad_segs=quad_segs,
        cap_style=cap_style,
        join_style=join_style,
        single_sided=True,
    )
    right_lane = buffer(
        centre_vertex,
        (-1 * buffer_size),
        quad_segs=quad_segs,
        cap_style=cap_style,
        join_style=join_style,
        single_sided=True,
    )

    # Get the external boundaries by removing those that are in the center_vertex
    left_lane_vertex = get_line_coords(get_coordinates(left_lane), original_vertex)
    right_lane_vertex = get_line_coords(get_coordinates(right_lane), original_vertex)[::-1]

    # Smooth and resample
    interpolated_left_lane_vertex = LineString(interpolate_and_resample(
        [(p[0], p[1]) for p in list(left_lane_vertex)], sampling_unit_m=sampling_unit_m
    ))

    interpolated_right_lane_vertex = LineString(interpolate_and_resample(
        [(p[0], p[1]) for p in list(right_lane_vertex)], sampling_unit_m=sampling_unit_m
    ))

    # Now project the points of the centre_vertex to left/right such that we get the same number of points
    projected_interpolated_left_lane_vertex = np.array([ (p.x, p.y) for p in [interpolated_left_lane_vertex.interpolate(interpolated_left_lane_vertex.project(Point(x))) for x in centre_vertex.coords]])
    projected_interpolated_right_lane_vertex = np.array([ (p.x, p.y) for p in [interpolated_right_lane_vertex.interpolate(interpolated_right_lane_vertex.project(Point(x))) for x in centre_vertex.coords]])
    

    assert len(projected_interpolated_left_lane_vertex) == len(projected_interpolated_right_lane_vertex), "Wrong number of points!"

    new_centre_vertex = create_centre_lanelet(projected_interpolated_left_lane_vertex, projected_interpolated_right_lane_vertex)

    def debug_plot():
        import matplotlib.pyplot as plt
        plt.plot(*left_lane.exterior.xy, ".-", color="green", alpha=0.5)
        plt.plot(projected_interpolated_left_lane_vertex[:, 0], projected_interpolated_left_lane_vertex[:, 1], "*-", color="green")

        plt.plot(*right_lane.exterior.xy, ".-", color="red", alpha=0.5)
        plt.plot(projected_interpolated_right_lane_vertex[:, 0], projected_interpolated_right_lane_vertex[:, 1], "*-", color="red")

        plt.plot(new_centre_vertex[:, 0], new_centre_vertex[:, 1], "*-", color="black")

        plt.gca().set_aspect("equal")
        plt.show()

    return projected_interpolated_left_lane_vertex, projected_interpolated_right_lane_vertex, new_centre_vertex


def create_lanelet_object(
    lanelet_id: int, left_vertices: np.ndarray, right_vertices: np.ndarray
) -> Lanelet:
    """Generates a Lanelet object based on left,right vertices of a road."""
    centre_vertices = create_centre_lanelet(left_vertices, right_vertices)
    # print(f'create lanelet object id: {id}')
    # print(f'left vert: {left_vertices}')
    # print(f'middle vert: {centre_vertices}')
    # print(f'right vert: {right_vertices}')
    lanelet = Lanelet(left_vertices, centre_vertices, right_vertices, lanelet_id, lanelet_type={LaneletType('highway')})
    return lanelet

from commonroad.scenario.scenario import Scenario
from utils import interpolate_and_resample

def generate_lanelet_network_from_vertex(scenario: Scenario, start_vertex: np.ndarray, lanelet_size: float= 3.0, sampling_unit_m : float = 2.0, interpolate_turns_with_pts: int = 8) -> LaneletNetwork:
    """Creates a lanelet network from a single vertex which includes 2 lanelets adjacent to each other. Each lanelet consists of left,centre,right vertex. Default width is 2."""

    # This somehow reduces the number of points, so we need to resample them... 
    # However, we need to ensure that joined segments have the same number of point?
    # Shall we project using the start_vertex instead of interpolating?
    
    left_lane_vertex_buffer, right_lane_vertex_buffer, centre_lane_vertex_buffer = (
        create_lanelet_vertices_buffer(start_vertex, buffer_size=lanelet_size, quad_segs=0, sampling_unit_m=sampling_unit_m)
    )

    def debug_plot():
        import matplotlib.pyplot as plt
        
        plt.plot(left_lane_vertex_buffer[:, 0], left_lane_vertex_buffer[:, 1], ".", zorder=100, color="blue")
        plt.plot(centre_lane_vertex_buffer[:, 0], centre_lane_vertex_buffer[:, 1], ".", zorder=100, color="green")
        plt.plot(right_lane_vertex_buffer[:, 0], right_lane_vertex_buffer[:, 1], ".", zorder=100, color="red")
        plt.plot([p[0] for p in start_vertex], [p[1] for p in start_vertex], "*", zorder=101, color="black")
        plt.gca().set_aspect('equal')
        plt.show()


    # Create lanelets
    lanelet_101_id = scenario.generate_object_id()
    lanelet_101 = create_lanelet_object(
        lanelet_101_id, centre_lane_vertex_buffer, right_lane_vertex_buffer
    )
    # The adjacent lane going the other way needs to have its array reversed to get the correct direction.
    lanelet_102_id = scenario.generate_object_id()
    lanelet_102 = create_lanelet_object(
        lanelet_102_id, centre_lane_vertex_buffer[::-1], left_lane_vertex_buffer[::-1]
    )
    lanelet_101.adj_left = lanelet_102_id
    lanelet_102.adj_left = lanelet_101_id

    # create lanelet network
    lanelet_network = LaneletNetwork()
    lanelet_network.add_lanelet(lanelet_101)
    lanelet_network.add_lanelet(lanelet_102)

    return lanelet_network


# if __name__ == "__main__":
#     ...
#     # *********************
#     # Plot a lanelet network
#     # *********************
#     from generate_simple_road import generate_trajectory
#     from shapely.geometry import Point

#     initial_location = Point(0, 0)
#     initial_rotation = 0
#     driving_actions = []
#     driving_actions.append(
#         {
#             "name": "follow",
#             "trajectory_segments": [
#                 {"type": "straight", "length": 20.0},
#                 {"type": "turn", "angle": -90.0, "radius": 40.0},
#                 {"type": "turn", "angle": +20.0, "radius": 100.0},
#             ],
#         }
#     )
#     res = generate_trajectory(initial_location, initial_rotation, driving_actions)
#     road_points = [[x1, y1] for x1, y1 in res]

#     # start_vertex = np.array([[0,0], [5,5], [10,5], [15,0]])
#     # start_vertex = np.array([[0,0], [5,0], [10,0], [15,0]])
#     # start_vertex = np.array([[0, 0], [1, 1], [3, 3], [5, 5], [7, 4], [8, 6]])
#     lanelet_network_vertex = generate_lanelet_network_from_vertex(road_points)
#     draw_lanelet_network(lanelet_network_vertex)
