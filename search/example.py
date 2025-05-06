# Force non-interactive mode
import matplotlib
matplotlib.use('Agg')

from matplotlib import pyplot as plt

from cr_integration.generate_scenario import generate_scenario_and_planning_problem_set_from_road_points, draw_scenario, write_scenario_to_file
from utils import generate_road_points

import numpy as np
import os

if __name__ == "__main__":
    # Generation configurations
    sampling_unit_m = 2.0
    interpolate_turns_with_pts = 100
    initial_location_coords = [0.0, 0.0]
    initial_rotation_deg = - 90.0

    # This example shows how to generate roads made of three segments:
    # The first segment (predix) and the last one (suffix) are straight segments with fixed length
    prefix = [{ "type": "straight", "length": 100.0 }]
    suffix = [{ "type": "straight", "length": 100.0 }]

    # Positive means left turns, negative right turns  
    scenarios_folder_name = "variable_radius"
    angle_degs = [-90.0, -135.0, 135.0, +90] # This is a Right turn
    radius_list = range(10, 200, 20)

    # scenarios_folder_name = "variable_angle"
    # angle_degs = list(range(-120, 120, 30)) # This is a Right turn
    # radius_list = [10, 50, 100, 150]

    for angle_deg in angle_degs:
        for radius_m in radius_list:
            central_segment = { 
                "type": "turn",
                "radius": radius_m,
                "angle": angle_deg
            }

            # Define a road as sequences of segments
            road_segments = prefix + [central_segment] + suffix

            # Generate the acutal road points, ensuring points are sampled uniformly every 2.0 meters
            road_points, _ = generate_road_points(
                initial_location_coords,
                initial_rotation_deg,
                road_segments,
                sampling_unit_m=sampling_unit_m, 
                interpolate_turns_with_pts=interpolate_turns_with_pts
            )

            # Render the road points into CommonRoad lanelets (one lanelet per side, like the SBFT competitions)
            road_vertex = [[x1, y1] for x1, y1 in road_points]

            # Generate a scenario containing the roads and the planning problem (initial and final states)
            # Pseudo random ID - we need one
            # from datetime import datetime
            # scenario_id = datetime.today().timestamp()
            scenario_id = radius_m
            scenario, planning_problem_set = generate_scenario_and_planning_problem_set_from_road_points(road_vertex, scenario_id,
                                                                                                        sampling_unit_m=sampling_unit_m, 
                                                                                                        interpolate_turns_with_pts=interpolate_turns_with_pts)

            # Now you can store the scenario to file or execute it with CommonRoad.
            # Probably storing to file is the safest so we can keep track of what we generate
            

            # search folder
            folder_containing_this_script = os.path.dirname(os.path.realpath(__file__))
            # project level folder
            one_above_the_folder_containing_this_script = os.path.dirname(folder_containing_this_script)
            
            # the default folder where to store stuff
            direction = "left" if angle_deg < 0 else "right"
            abs_value_angle = abs(angle_deg)
            output_folder = os.path.join(one_above_the_folder_containing_this_script, f"{scenarios_folder_name}")
            # , f"angle_deg_{abs_value_angle}_{direction}")
            scenario_name = f"angle_deg_{abs_value_angle}_{direction}_radius_m_{radius_m}"
            # f"scenario_{scenario_id}.xml"
            scenario_file = os.path.join(output_folder, f"{scenario_name}.xml")
            scenario_file_png = os.path.join(output_folder, f"{scenario_name}.png")

            print(f"Output scenario: {scenario_file}")

            os.makedirs(output_folder, exist_ok=True)
            write_scenario_to_file(scenario, planning_problem_set, scenario_file)

            fig = draw_scenario(scenario, planning_problem_set)
            fig.savefig(scenario_file_png)
            plt.close()