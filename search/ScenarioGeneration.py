from pymoo.algorithms.base.genetic import GeneticAlgorithm
from pymoo.core.problem import ElementwiseProblem
from pymoo.termination.max_gen import MaximumGenerationTermination

from cr_beamng_cosimulation import commands as simulation_commands
from search.cr_integration.generate_scenario import generate_scenario_and_planning_problem_set_from_road_points, \
    write_scenario_to_file, draw_scenario
from search.utils import generate_road_points
from datetime import datetime
import os


class GenerationProblem(ElementwiseProblem):
    def __init__(self, **kwargs):
        super().__init__(n_var=2, n_obj=1, n_ieq_constr=0, **kwargs)
        self.lower_bound = [1, -90] #TODO fix the boundaries
        self.upper_bound = [50, 90] #TODO fix the boundaries

    def _evaluate(self, x, out, *args, **kwargs):
        radius_m = x[0]
        angle_deg = x[1]
        scenario_name = self._generate_scenario(radius_m, angle_deg)

        simulation_sim, evaluation_sim = simulation_commands.simulate(scenario_name)
        simulation_cosim, evaluation_cosim = simulation_commands.cosimulate(scenario_name)

        diff_results = 42 #TODO compute the difference of results
        
        out["F"] = [diff_results]
        print(out["F"])

    def _generate_scenario(self, radius_m=50, angle_deg=-30.0):
        # Generation configurations
        sampling_unit_m = 2.0
        interpolate_turns_with_pts = 100
        initial_location_coords = [0.0, 0.0]
        initial_rotation_deg = - 90.0

        # This example shows how to generate roads made of three segments:
        # The first segment (predix) and the last one (suffix) are straight segments with fixed length
        prefix = [{"type": "straight", "length": 50.0}]
        suffix = [{"type": "straight", "length": 50.0}]

        # The central segment can be another straight or a turn
        #radius_m = 50.0
        # Positive means left turns, negative right turns
        #angle_deg = -30.0  # This is a Right turn

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

        scenario_id = datetime.today().timestamp()
        scenario, planning_problem_set = generate_scenario_and_planning_problem_set_from_road_points(road_vertex,
                                                                                                     scenario_id,
                                                                                                     sampling_unit_m=sampling_unit_m,
                                                                                                     interpolate_turns_with_pts=interpolate_turns_with_pts)

        # Now you can store the scenario to file or execute it with CommonRoad.
        # Probably storing to file is the safest so we can keep track of what we generate

        # search folder
        folder_containing_this_script = os.path.dirname(os.path.realpath(__file__))
        # project level folder
        one_above_the_folder_containing_this_script = os.path.dirname(folder_containing_this_script)
        # the default folder where to store stuff
        output_folder = os.path.join(one_above_the_folder_containing_this_script, "temp-generation")

        scenario_file = os.path.join(output_folder, f"scenario_{scenario_id}.xml")
        print(f"Output scenario: {scenario_file}")
        os.makedirs(output_folder, exist_ok=True)
        write_scenario_to_file(scenario, planning_problem_set, scenario_file)

        #scenario_file_png = os.path.join(output_folder, f"scenario_{scenario_id}.png")
        #fig = draw_scenario(scenario, planning_problem_set)
        #fig.savefig(scenario_file_png)
        return scenario_file


if __name__ == '__main__':
    algorithm = GeneticAlgorithm(pop_size=10)
    problem = GenerationProblem()
    termination_criterion = MaximumGenerationTermination(10)
    algorithm.setup(problem, termination=termination_criterion, save_history=True, verbose=True)