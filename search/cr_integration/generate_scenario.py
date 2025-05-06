import numpy as np
import math
import matplotlib.pyplot as plt

from typing import Tuple, List, Dict

from shapely import Point, LineString, get_coordinates

from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, Rectangle
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import CustomState, InitialState
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import resample_polyline, chaikins_corner_cutting
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.util import AngleInterval, Interval
from commonroad.visualization.mp_renderer import MPRenderer

from cr_integration.generate_lanelet import generate_lanelet_network_from_vertex

# from self_driving.road_polygon import RoadPolygon
from commonroad.common.common_lanelet import LaneletType
from commonroad.common.file_writer import XMLFileWriter, OverwriteExistingFile


# from src.generate_road_test import (
#     generate_road_test_from_vertex,
#     generate_road_test_one_test,
#     generate_road_test_deepjanus,
# )
# from code_pipeline.tests_generation import RoadTestFactory
# from src.utils import *


# # TODO: need to generate IDs dynamically
# def generate_lanelet_network_from_road_test(
#     road_test: RoadTestFactory.RoadTest,
# ) -> LaneletNetwork:
#     """Takes a RoadTest object which includes road vertices and outputs a LaneletNetwork Object. It is assumed the RoadTest Object is a single road with adjacent lanes in opposite directions."""
#     road_polygon = road_test.get_road_polygon()

#     lanelet_101 = Lanelet(
#         (np.array(road_polygon.road_points.middle)[:, :2]),
#         get_coordinates(road_polygon.right_polyline),
#         np.array(road_polygon.road_points.right),
#         101,
#         lanelet_type={LaneletType("highway")},
#     )
#     # data points for adjacent lane should be inverted to identify correct direction.
#     lanelet_102 = Lanelet(
#         (np.array(road_polygon.road_points.left))[::-1],
#         get_coordinates(road_polygon.left_polyline)[::-1],
#         (np.array(road_polygon.road_points.middle)[:, :2])[::-1],
#         102,
#         lanelet_type={LaneletType("highway")},
#     )
#     # TODO: can I use static lanelet names or should they be unique across all scenarios?
#     lanelet_101.adj_left = 102
#     lanelet_102.adj_left = 101

#     # create lanelet network
#     lanelet_network = LaneletNetwork()
#     lanelet_network.add_lanelet(lanelet_101)
#     lanelet_network.add_lanelet(lanelet_102)

#     return lanelet_network


def generate_scenario_and_planning_problem_set_from_road_points(road_vertex: List[Tuple[float, float]], 
                                                                scenario_id: int,
                                                                dt: float = 0.1,
                                                                sampling_unit_m : float = 2.0, interpolate_turns_with_pts: int = 8) -> Tuple[Scenario, PlanningProblemSet]:
    """
    Generate the commonroad scenario and planning problem from the given road 
    """

    scenario = Scenario(dt) #, scenario_id=ScenarioID(scenario_version=scenario_id))
    # return scenario
    # Generate the road netwok
    lanelet_network = generate_lanelet_network_from_vertex(scenario, np.array(road_vertex, dtype=np.float32), 
                                                           sampling_unit_m=sampling_unit_m, interpolate_turns_with_pts=interpolate_turns_with_pts)
    scenario.replace_lanelet_network(lanelet_network)
    # Generate the scenario
    # scenario = generate_scenario(dt, lanelet_network, scenario_id)
    # Use the scenario to generate the planning problems
    planning_problem = generate_planning_problem(scenario)
    planning_problem_set = PlanningProblemSet([planning_problem])

    return scenario, planning_problem_set


def generate_planning_problem(scenario: Scenario, initial_offset_m: float= 10, goal_offset_m: float = 10, max_duration_steps: int = 1000) -> PlanningProblem:
    """
    Return a PlanningProblem for a lanelet that includes initial state and goal region

    :param simple_road: for generating roads used in experiments, I want the goal region to be some distance from the end of the road vertex.
                        This flag indicates that the last part of the road will have sufficient distance to put the goal state much earlier.
    """
    planning_problem_id = scenario.generate_object_id()
    # Find the initial lanelet and place the initial state 10 meters in
    initial_state = generate_initial_state(scenario, initial_offset_m)
    # Fins the last lanelet and place the goal area 10 meters in
    goal_region = generate_goal(scenario, initial_state,  goal_offset_m, max_duration_steps)
    return PlanningProblem(planning_problem_id, initial_state, goal_region)


# Function credit: https://anonymous.4open.science/r/evita-89F2/code/evita/generators/utils.py
def cut(line: LineString, distance: float):
    # cuts a line in two at distance from its starting point
    if distance <= 0.0 or distance >= line.length:
        return [line]
    coords = list(line.coords)
    for i, p in enumerate(coords):
        pd = line.project(Point(p))
        if pd == distance:
            return [LineString(coords[: i + 1]), LineString(coords[i:])]
        if pd > distance:
            cp = line.interpolate(distance)
            return [
                LineString(coords[:i] + [(cp.x, cp.y)]),
                LineString([(cp.x, cp.y)] + coords[i:]),
            ]

# Function credit: https://anonymous.4open.science/r/evita-89F2/code/evita/generators/utils.py
def get_orientation_by_coords(
    first_point: Tuple[float, float], next_point: Tuple[float, float]
):
    a_x, a_y = next_point
    b_x, b_y = first_point
    # compute orientation: https://stackoverflow.com/questions/42258637/how-to-know-the-angle-between-two-vectors
    return math.atan2(a_y - b_y, a_x - b_x)

def generate_initial_state(scenario: Scenario, initial_offset_m: float) -> InitialState:
    """Return an InitialState object for a lanelet as starting object for ego vehicle."""

    # TODO: We should consider the lanelet that corresponds to the initial road point...
    # We assume there's only one ROAD. We stop as soon as we find an "entry" lanelet
    # We sort lanelets by ID
    initial_lanelet = None
    for lanelet in sorted(scenario.lanelet_network.lanelets, key= lambda l : l.lanelet_id):
        if len(lanelet.predecessor) == 0:
            initial_lanelet = lanelet
            break

    # We place the initial state at initial_offset_m meters from the start of the road
    # if the lanelet is too short we move to the next one or fail if none is there
    while max(initial_lanelet.distance) < initial_offset_m:
        # Reduce the offset by the lenght of the lanelet
        initial_offset_m = initial_offset_m - initial_lanelet.distance
        # This will fail if theres no successor
        initial_lanelet = scenario.lanelet_network.find_lanelet_by_id(initial_lanelet.successor[0])
    
    # Find the actual location of the initial position
    center_vertices = LineString((initial_lanelet.center_vertices))   
    start_center_split, end_center_split = cut(center_vertices, initial_offset_m)
    
    start_center = Point(start_center_split.coords[-1])
    # It cannot be [0] otherwise it's the same as the start_center
    next_point = Point(end_center_split.coords[1])

    def debug_plot():
        import matplotlib.pyplot as plt
        from commonroad.visualization.mp_renderer import MPRenderer
        rnd = MPRenderer()
        scenario.draw(rnd)
        rnd.render()

        plt.plot(*center_vertices.xy, "s", zorder=100, color="blue")
        plt.plot(*start_center_split.xy, ".", zorder=100, color="green")
        plt.plot(start_center.x, start_center.y, "o", ms=10, zorder=100)

        plt.show()

    orientation = get_orientation_by_coords((start_center.x, start_center.y), (next_point.x, next_point.y))
    position = np.array([start_center.x, start_center.y])

    # velocity = 10.0
    # acceleration = 1.0
    # yaw_rate = 0.0
    # slip_angle = 0.0
    
    # Initial state must be stationary as this is more realistic
    velocity = 0.0
    acceleration = 0.0
    yaw_rate = 0.0
    slip_angle = 0.0
    return InitialState(
        time_step=0,
        position=position,
        orientation=orientation,
        velocity=velocity,
        acceleration=acceleration,
        yaw_rate=yaw_rate,
        slip_angle=slip_angle,
    )


def get_possible_goal_for_lanelet(scenario: Scenario, initial_state: InitialState, goal_offset_m: float) -> Rectangle:
    """
    Return the Rectangle corresponding to the Goal Area for the given lanelet which must be goal_offset_m before the end of the lanelet
    """
    # if simple_road:
    #     # reversed_center_vertices = LineString((lanelet.center_vertices[:-1])[::-1])
    #     reversed_center_vertices = LineString((lanelet.center_vertices)[::-1])
    # # reverse center vertices to get the distance from lanelet end with cut()
    # elif len(lanelet.center_vertices) < 6:
    #     reversed_center_vertices = LineString((lanelet.center_vertices)[::-1])
    # else:
    #     reversed_center_vertices = LineString((lanelet.center_vertices[:-1])[::-1])

    # Find an exit lanelet reacheable from the initial lanelet (assume only one successor, no circle)

    def debug_plot():
        import matplotlib.pyplot as plt
        from commonroad.visualization.mp_renderer import MPRenderer
        rnd = MPRenderer()
        scenario.draw(rnd)
        initial_state.draw(rnd)
        rnd.render()
        plt.show()

    # debug_plot()
        
    # This is a nested list
    initial_lanelet_id = scenario.lanelet_network.find_lanelet_by_position([initial_state.position])[0][0]
    exit_lanelet = scenario.lanelet_network.find_lanelet_by_id(initial_lanelet_id)
    while len(exit_lanelet.successor) > 0:
        exit_lanelet = scenario.lanelet_network.find_lanelet_by_id(exit_lanelet.successor(0))

    # Check if the exit lanelet is long enough
    while max(exit_lanelet.distance) < goal_offset_m:
        # Reduce the offset
        goal_offset_m = goal_offset_m - max(exit_lanelet.distance)
        # Use the predecessor
        exit_lanelet = scenario.lanelet_network.find_lanelet_by_id(exit_lanelet.predecessor[0])
    

    reversed_center_vertices = LineString((exit_lanelet.center_vertices[:-1])[::-1])
    goal_center_vertices, previous_center_vertices = cut(reversed_center_vertices, goal_offset_m)
    goal_center = Point(goal_center_vertices.coords[-1])
    # It cannot be [0] otherwise it is the same as the goal center
    previous_point = Point(previous_center_vertices.coords[1])

    def debug_plot():
        import matplotlib.pyplot as plt
        plt.plot(*goal_center_vertices.xy, ".", color="red")
        plt.plot(goal_center.x, goal_center.y, "*", color="black")
        plt.plot(*previous_center_vertices.xy, ".", color="blue")
        plt.plot(previous_point.x, previous_point.y, "s", color="black", alpha=0.5)
        plt.show()
   
    # TODO: assert distance_from_start(initial_lanelet, initial_position) < distance_from_start(initial_lanelet, goal_position) < 
    orientation = get_orientation_by_coords(
        (goal_center.x, goal_center.y), (previous_point.x, previous_point.y)
    )

    # TODO Move to configuration the size of the goal area
    length = 10
    width = 3

    # TODO This should be the lanelet boundary instead...
    return Rectangle(length, width, np.array([goal_center.x, goal_center.y]), orientation)


def generate_goal(scenario: Scenario, initial_state: InitialState, goal_offset_m: float, max_duration_steps: int) -> GoalRegion:
    """
    Return a well formed Goal State object from the rectangle. By default, AV must reach the goal area
    before the end of the scenario
    """
    goal_area = get_possible_goal_for_lanelet(scenario, initial_state, goal_offset_m)
    goal_state_list = [
        CustomState(position=goal_area, time_step=Interval(0, max_duration_steps)) #, velocity= Interval(0,30))
    ]
    return GoalRegion(goal_state_list)


def draw_scenario(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    failure_point: Tuple[float, float] = None,
) -> None:
    """Plots a scenario and includes a red X for point of failure if defined."""
    fig, ax = plt.subplots(figsize=(16, 9))
    
    # Render the scenario and planning problems
    rnd = MPRenderer(ax=ax)
    scenario.draw(rnd)
    planning_problem_set.draw(rnd)
    rnd.render()
    
    # TODO Add additional markups, like segment boundaries if any
    
    return fig


def write_scenario_to_file(scenario: Scenario, planning_problem_set: PlanningProblemSet, output_file: str) -> None:
    """Function to write a scenario and planning_problem_set to xml in commonroad io format."""
    author = "CONDI"
    affiliation = ""
    source = ""
    tags = []

    # write new scenario
    fw = XMLFileWriter(
        scenario,
        planning_problem_set,
        author,
        affiliation,
        source,
        tags,
        decimal_precision=10,
    )

    fw.write_to_file(output_file, OverwriteExistingFile.ALWAYS)

