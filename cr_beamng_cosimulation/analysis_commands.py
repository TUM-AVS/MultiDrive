#
# NOTE: If a vehicle crashes it will have fewer observations, so the box plots will be skewed!
#


import click
import os
import traceback
import csv
import pickle

import cr_scenario_handler.utils.multiagent_helpers as hf

from beamngpy import BeamNGpy, Vehicle as BeamngVehicle, Scenario as BeamngScenario, Road as BeamngRoad
from beamngpy.sensors import Electrics, Damage, Timer
from beamngpy.types import Float3

from commonroad.common.reader.file_reader_xml import XMLFileReader as CR_FileReader
from commonroad.common.writer.file_writer_xml import XMLFileWriter as CR_FileWriter

from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem as CR_PlanningProblem, PlanningProblemSet as CR_PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario as CR_Scenario
from commonroad.scenario.state import State as CR_State, InputState as CR_InputState
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.trajectory import Trajectory as CR_Trajectory
from vehiclemodels.vehicle_parameters import setup_vehicle_parameters

from copy import deepcopy

from cr_beamng_cosimulation.beamng_scenario_handler import BeamNGScenarioHandler
from cr_beamng_cosimulation.controller import BngController
from cr_beamng_cosimulation.utils import get_configuration_dict
from cr_beamng_cosimulation.utils import is_standing_state, get_state_of_beamng_vehicles
from cr_beamng_cosimulation.utils import get_beamng_configuration, get_beamng_vehicle_models_configuration
from cr_beamng_cosimulation.beamng_simulation import BeamNGSimulation as BNG_Simulation
from cr_beamng_cosimulation.runtime_monitoring import TrajectoryTrackingError
from cr_beamng_cosimulation.utils import FRENETIX_MODULE_PATH

from cr_scenario_handler.simulation.simulation import Simulation as CR_Simulation
from cr_scenario_handler.planner_interfaces.frenet_interface import FrenetPlannerInterface
from cr_scenario_handler.utils.configuration import SimConfiguration, FrenetConfiguration
from cr_scenario_handler.utils.configuration_builder import ConfigurationBuilder
from cr_scenario_handler.utils.helper_functions import find_lanelet_by_position_and_orientation
from cr_scenario_handler.evaluation.simulation_evaluation import evaluate_simulation

from datetime import datetime

from frenetix_motion_planner.state import ReactivePlannerState

from omegaconf.omegaconf import OmegaConf, DictConfig

from typing import List, Dict, Any, Tuple
import shutil

#########
# UTILITIES
#########
def _setup_output_folders(output_folder: str):
    
    # if os.path.exists(output_folder):
    #     while True:
    #         answer = input(f"Remove folder {output_folder}. Continue? [y/n]  ")
    #         if answer.lower() in ["y","yes"]:
    #             shutil.rmtree(output_folder)
    #             break
    #         elif answer.lower() in ["n","no"]:
    #             assert False
    # print(f"Setting up folder {os.path.abspath(output_folder)}")
    os.makedirs(output_folder, exist_ok=True)

import sqlite3
import pandas as pd

def _get_agents_id(simulation_db: str):
    con = sqlite3.connect(simulation_db, isolation_level="EXCLUSIVE")
    con.executescript("""
        PRAGMA journal_mode = OFF;
        PRAGMA temp_store = MEMORY;
    """)
    con.commit()

    res = con.execute(f"SELECT DISTINCT(agent_id) FROM scenario_evaluation;")
    rows = res.fetchall()

    return [tup[0] for tup in rows]
    
def _get_last_timestep_for_agent(simulation_db, agent_id):
    con = sqlite3.connect(simulation_db, isolation_level="EXCLUSIVE")
    con.executescript("""
        PRAGMA journal_mode = OFF;
        PRAGMA temp_store = MEMORY;
    """)
    con.commit()

    # This is the last timestep for this agent in this scenario
    res = con.execute(f"SELECT last_timestep FROM results WHERE agent_id == {agent_id};")
    row = res.fetchone()

    return row[0]

def _extract_agents_trace_from_logs(simulation_logs: str):
    simulation_db = os.path.join(simulation_logs, "simulation.db")
    # get the number and id of the agents
    # for each agent, read the logs.csv file from their folder
    agents_trace = dict()
    for agent_id in _get_agents_id(simulation_db):
        agent_logs_csv_file = os.path.join(simulation_logs, str(agent_id), "logs.csv")
        agent_df = pd.read_csv(agent_logs_csv_file, sep=";")
        # Filter only the relevant columns
        # TODO Are the positions shifted to the center?
        agent_df = agent_df[ ["trajectory_number", "x_position_vehicle_m", "y_position_vehicle_m"] ]
        # Rename the columns
        agent_df = agent_df.rename(
            columns={
                'trajectory_number': 'time_step',
                'x_position_vehicle_m': 'position_x',
                'y_position_vehicle_m': 'position_y'
                })
        # Convert to list of namedtuples
        agents_trace[agent_id] = list(agent_df.itertuples(name='AgentSample', index=False))

    return agents_trace

import itertools
from commonroad.visualization.mp_renderer import MPRenderer, MPDrawParams
from cr_beamng_cosimulation.utils import get_color_dict_for_planning_problems
import matplotlib.pyplot as plt

def __lighten_color(color, amount=0.5):
    # SEE: https://stackoverflow.com/questions/37765197/darken-or-lighten-a-color-in-matplotlib
    """
    Lightens the given color by multiplying (1-luminosity) by the given amount.
    Input can be matplotlib color string, hex string, or RGB tuple.

    Examples:
    >> lighten_color('g', 0.3)
    >> lighten_color('#F034A3', 0.6)
    >> lighten_color((.3,.55,.1), 0.5)
    """
    import matplotlib.colors as mc
    import colorsys
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])

def _generate_plot(scenario: CR_Scenario, planning_problem_set: CR_PlanningProblemSet,
                   scenario_tuples_dict):
                #    simulated_scenario: CR_Scenario, 
                #    cosimulated_scenario_with_bng: CR_Scenario,cosimulated_scenario_with_cr: CR_Scenario,
                #    resimulated_scenario: CR_Scenario, 
                #    simulated_agent_traces: Dict[int, List[Tuple]], 
                #    cosimulated_agent_traces_with_bng: Dict[int, List[Tuple]], cosimulated_agent_traces_with_cr: Dict[int, List[Tuple]], 
                #    resimulated_agent_traces: Dict[int, List[Tuple]]):
    
    rnd = MPRenderer(figsize=(20, 20))
    scenario.draw(rnd)
    # TODO Make it parametric, using a standard palette
    colors = ["green", "red", "yellow", "blue"]
    # TODO Make it parametric, using the actual key
    # labels = ["simulation", "cosimulation (bng)", "cosimulation (cr)", "resimulation"]

    # One plot every 5 seconds
    plot_every_steps = 50 

    # Draw Occupancy of the vehicles using the appropriate colors.
    # for scenario, color in zip([simulated_scenario, cosimulated_scenario_with_bng, cosimulated_scenario_with_cr, resimulated_scenario], colors):
    for scenario_key, color in zip(sorted(scenario_tuples_dict.keys()), colors):
        
        scenario = scenario_tuples_dict[scenario_key][1]

        for obstacle in scenario.obstacles:
            last_time_step = obstacle.prediction.trajectory.state_list[-1].time_step
            for time_step in range(0,obstacle.prediction.trajectory.state_list[-1].time_step, plot_every_steps):
                if obstacle.occupancy_at_time(time_step) is not None:
                    # Set obstacle parameters
                    obs_params = MPDrawParams()
                    # Cannot use icon if you need transparent background
                    obs_params.dynamic_obstacle.draw_icon = False
                    obs_params.dynamic_obstacle.show_label = True
                    
                    # We need both otherwise begin and end otherwise it will not show it
                    obs_params.dynamic_obstacle.time_begin = time_step
                    obs_params.dynamic_obstacle.time_end = last_time_step

                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = color
                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "black"
                    # Ensure this is draw above everything else
                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.zorder = 100
                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.opacity = 0.2

                    # Finally render it
                    obstacle.draw(rnd, draw_params=obs_params)

    # Render the scenario onto the plot
    rnd.render()

    # Sort and assign colors based on agents_id
    # color_map = get_color_dict_for_planning_problems(planning_problem_set)

    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')
    
    for lanelet in scenario.lanelet_network.lanelets:
        for vertices in [lanelet.left_vertices, lanelet.center_vertices, lanelet.right_vertices]:
            min_x_val_vertices, max_x_val_vertices = np.min(vertices[:,0]), np.max(vertices[:,0])
            min_y_val_vertices, max_y_val_vertices = np.min(vertices[:,1]), np.max(vertices[:,1])

            min_x = min([min_x, min_x_val_vertices])
            max_x = max([max_x, max_x_val_vertices])
            min_y = min([min_y, min_y_val_vertices])
            max_y = max([max_y, max_y_val_vertices])

    margin = 10

    x_lim = (min_x - margin, max_x + margin)
    y_lim = (min_y - margin, max_y + margin)

    # Plots all the positions
    # for executed_scenario_traces, label, color in zip([simulated_agent_traces, cosimulated_agent_traces_with_bng, cosimulated_agent_traces_with_cr, resimulated_agent_traces], labels, colors):
    for executed_scenario_traces_key, color in zip(sorted(scenario_tuples_dict.keys()), colors):
        label = executed_scenario_traces_key
        executed_scenario_traces = scenario_tuples_dict[executed_scenario_traces_key][0]
        # The trajectories are different because their speed is different
        for agent_id in executed_scenario_traces.keys():
            agent_trace = executed_scenario_traces[agent_id]
            x1 = [t.position[0] for t in agent_trace]
            y1 = [t.position[1] for t in agent_trace]
            plt.plot(x1, y1, ".-", color=color, zorder=100, alpha=0.3, lw=2, label=f"{label}")

    
    plt.legend(prop={'size': 24})

    # Set plots limits so we show only the relevant part of the road
    plt.xlim(x_lim)
    plt.ylim(y_lim)

def _plot_agents_trajectories(output_folder: str, 
                           scenario: CR_Scenario, planning_problem_set: CR_PlanningProblemSet,
                           scenario_tuples_dict):
                        #    simulated_scenario: CR_Scenario, 
                        #    cosimulated_scenario_with_bng: CR_Scenario, cosimulated_scenario_with_cr: CR_Scenario,
                        #    resimulated_scenario: CR_Scenario,
                        #    simulated_agent_traces: Dict[int, List[Tuple]], 
                        #    cosimulated_agent_traces_with_bng: Dict[int, List[Tuple]], cosimulated_agent_traces_with_cr: Dict[int, List[Tuple]], 
                        #    resimulated_agent_traces: Dict[int, List[Tuple]]):
    """
    Generate the following plots:
    - overall.png showing the trajectory of all the agents in all simulations (use different colors)
    - agent_{agent_id}.png showring the trajectory of agent_id in all simulations (use different colors)
    """

    # Since we have many comparisons to do we need to separate them in different folders
    os.makedirs(output_folder, exist_ok=True)

    _generate_plot(scenario, planning_problem_set, scenario_tuples_dict)
                #    simulated_scenario, cosimulated_scenario_with_bng, cosimulated_scenario_with_cr, resimulated_scenario,
                #    simulated_agent_traces, cosimulated_agent_traces_with_bng, cosimulated_agent_traces_with_cr, resimulated_agent_traces)
    
    plt.savefig(os.path.join(output_folder, "overall.png"))

    # No need to replot for each agent
    # for agent_id in planning_problem_set.planning_problem_dict.keys():
    #     simulated_agent_trace = simulated_agent_traces[agent_id]
    #     cosimulated_agent_trace = cosimulated_agent_traces[agent_id]
    #     resimulated_agent_trace = resimulated_agent_traces[agent_id]

    #     _generate_plot(scenario, planning_problem_set, {agent_id: simulated_agent_trace}, {agent_id: cosimulated_agent_trace}, {agent_id: resimulated_agent_trace})

    #     plt.savefig(os.path.join(output_folder, f"agent_{agent_id}.png"))

    plt.clf()
    plt.close()

def _extract_agents_trace(scenario_file: str, simulated_scenario_file: str):
    # Load the original scenario to access the agents id
    _, planning_problem_set = CR_FileReader(scenario_file).open()

    # Load the simulation scenario to access the agents trajectories
    scenario, _ = CR_FileReader(simulated_scenario_file).open()
    
    # Accumulate the recorded states for each agent, but ensure we consider ONLY the actual observed states
    # and not the entire trajectories
    agents_trace = dict()
    for agent_id in planning_problem_set.planning_problem_dict.keys():
        # Get the scenario duration for the agent from the db
        # Assume that the content of trajectory is ONLY the actually simulated states, not the one planned!
        # last_timestep = _get_last_timestep_for_agent(simulation_db, agent_id)
        agent_trajectory = scenario.obstacle_by_id(agent_id).prediction.trajectory
        agents_trace[agent_id] = agent_trajectory.state_list[:]

    return agents_trace

import numpy as np
import csv
from shapely.geometry import Point

def _plot_bars(values: List[float]):
    plt.figure()
    plt.bar(list(range(0, len(values))), values)

# https://stackoverflow.com/questions/48298009/can-i-give-a-click-option-another-name
# This is taken from the Autoware CommonRoad paper repo, so we can use the same exact code to compute the errors
import math
def element_wise_euclideian_distance(position_agent_1: List[Tuple[float, float]], position_agent_2:List[Tuple[float, float]]):
    # Because vehicles move at different speed there might be a different number of observations
    min_len = min(len(position_agent_1), len(position_agent_2))
    position_agent_1 = position_agent_1[:min_len]
    position_agent_2 = position_agent_2[:min_len]

    # For each pair compute the distance between the points they represent
    distances = list()
    for p1, p2 in [(Point(t1), Point(t2)) for t1, t2 in zip(position_agent_1, position_agent_2)]:
        distances.append(p1.distance(p2))

    return distances

def element_wise_difference(value_list_1: List[float], value_list_2: List[float]):
    # Because vehicles move at different speed there might be a different number of observations
    min_len = min(len(value_list_1), len(value_list_2))
    value_list_1 = value_list_1[:min_len]
    value_list_2 = value_list_2[:min_len]

    # For each pair compute the distance between the points they represent
    differences = [ v1 - v2 for v1, v2 in zip(value_list_1, value_list_2)]
    return differences


def _compute_planned_driven_errors(planned_state, driven_state) -> Tuple[float, float, float]:
    """
    Computes error between planned and driven states
    """
    position_error: float = math.sqrt(
        (planned_state.position[0] - driven_state.position[0]) ** 2 +
        (planned_state.position[1] - driven_state.position[1]) ** 2
    )

    orientation_error: float = planned_state.orientation - driven_state.orientation
    velocity_error: float = planned_state.velocity - driven_state.velocity

    return position_error, orientation_error, velocity_error

@click.command()
@click.pass_context
@click.option('--executed-scenario-folder', "simulations_to_compare", type=click.Path(exists=True), multiple=True)
def compare(ctx, simulations_to_compare: List[str]):
    """
    Compute the difference of the aligned trajectories between low- and high-fidelity simulations following Wuersching et al., IV 2024
    """
    # Prepare the output folder where to store results and plots
    output_folder = ctx.obj["output_folder"]
    _setup_output_folders(output_folder)

    data_of_simulations = list()
    for simulation_folder in simulations_to_compare:
        print(f"Loading data for scenario {simulation_folder}")
        simulation_data = dict()
        simulation_data["simulation_folder"] = simulation_folder
        simulation_data["scenario_name"] = os.path.basename(simulation_folder)
        simulation_data["input_scenario_file"] = os.path.join(simulation_folder, "scenario_to_simulate.xml")
        #
        if os.path.exists(os.path.join(simulation_folder, "simulated_scenario.xml")):
            simulation_data["simulated_scenario_file"] = os.path.join(simulation_folder, "simulated_scenario.xml")
            # Extract the trace from the simulated scenario file where each agent is represented as an obstacle
            simulation_data["simulated_agents_trace"] = _extract_agents_trace(
                simulation_data["input_scenario_file"],
                simulation_data["simulated_scenario_file"]
            )

        if os.path.exists(os.path.join(simulation_folder, "cosimulated_scenario_with_bng.xml")):
            simulation_data["cosimulated_with_bng_scenario_file"] = os.path.join(simulation_folder, "cosimulated_scenario_with_bng.xml")
            simulation_data["cosimulated_with_bng_agents_trace"] = _extract_agents_trace(
                simulation_data["input_scenario_file"],
                simulation_data["cosimulated_with_bng_scenario_file"]
            )
    
        if os.path.exists(os.path.join(simulation_folder, "cosimulated_scenario_with_cr.xml")):
            simulation_data["cosimulated_with_cr_scenario_file"] = os.path.join(simulation_folder, "cosimulated_scenario_with_cr.xml")
            simulation_data["cosimulated_with_cr_agents_trace"] = _extract_agents_trace(
            simulation_data["input_scenario_file"],
            simulation_data["cosimulated_with_cr_scenario_file"]
        )
            
        if os.path.exists(os.path.join(simulation_folder, "resimulated_scenario.xml")):
            simulation_data["resimulated_scenario_file"] = os.path.join(simulation_folder, "resimulated_scenario.xml")
            simulation_data["resimulated_agents_trace"] = _extract_agents_trace(
            simulation_data["input_scenario_file"],
            simulation_data["resimulated_scenario_file"]
        )

        data_of_simulations.append(simulation_data)

    # For each scenario, do the comparison
    for simulation_data in data_of_simulations:
        # Load the original scenario and planning problems
        scenario, planning_problem_set = CR_FileReader(simulation_data["input_scenario_file"]).open()
        
        # TODO: now that we have the "executed" scenarios, we can simply rely on the MPRender to render (using alpha=0.5) both scenarios
        # with the second scenario using a little darker colors!
        # Compare includes:
        #   - Plotting the various scenarios on top of each other
        scenario_tuples_dict = dict()
        for sim in ["simulated", "cosimulated_with_bng", "cosimulated_with_cr", "resimulated"]:
            if f"{sim}_agents_trace" in simulation_data:
                traces = simulation_data[f"{sim}_agents_trace"]
                scenario, _ = CR_FileReader(simulation_data[f"{sim}_scenario_file"]).open()
                scenario_tuples_dict[sim] = (traces, scenario)

        plots_folder = os.path.join(output_folder, f"plots")
        _plot_agents_trajectories(plots_folder, scenario, planning_problem_set, scenario_tuples_dict)
                                #   simulated_scenario, cosimulated_scenario_with_bng, cosimulated_scenario_with_cr, resimulated_scenario,
                                #   simulated_agent_traces, cosimulated_with_bng_agent_traces, cosimulated_with_cr_agent_traces, resimulated_agent_traces)
        
        # TODO Move the following into a new method

        # Compare all the scenarios/sim against the original simulation
        # ERROR Metrics and error bars
        # For each planner in the simulations compute the errors between the original simulation and the actual one
        simulated_scenario, _ = CR_FileReader(simulation_data["simulated_scenario_file"]).open()
        
        # Collect errors between simulation and cosimulations if they are present
        # simulation_vs_cosimulation_with_bng = list()
        # simulation_vs_cosimulation_with_cr = list()
        # simulation_vs_resimulation = list()

        errors_dict = dict()

        for sim in ["cosimulated_with_bng", "cosimulated_with_cr", "resimulated"]:
            if f"{sim}_agents_trace" not in simulation_data:
                continue

            error_key =  f"simulated_vs_{sim}"
            other_scenario, _ = scenario, _ = CR_FileReader(simulation_data[f"{sim}_scenario_file"]).open()
            errors_dict[error_key] = list()

            for obstacle_id in [o.obstacle_id for o in simulated_scenario.obstacles]:

                # Co simulations
                for time_step in range(simulated_scenario.obstacle_by_id(obstacle_id).prediction.trajectory.state_list[-1].time_step):
                    planned_state = simulated_scenario.obstacle_by_id(obstacle_id).state_at_time(time_step)
                    driven_state = other_scenario.obstacle_by_id(obstacle_id).state_at_time(time_step)

                    if planned_state is None or driven_state is None:
                        continue

                    position_error, orientation_error, velocity_error = _compute_planned_driven_errors(planned_state, driven_state)
                    # TODO How is this possible?
                    # assert planned_state.time_step == driven_state.time_step, f"Not the same timestep {planned_state.time_step} == {driven_state.time_step}!"
                    errors_dict[error_key].append({
                        "agent_id": obstacle_id, 
                        "time_step": planned_state.time_step,
                        "position_error": position_error,
                        "orientation_error": orientation_error,
                        "velocity_error": velocity_error
                    })

        for file_name, values in errors_dict.items():
            # Store to file            
            simulation_vs_other_file_name = os.path.join(output_folder, f"{file_name}.csv")
            with open(simulation_vs_other_file_name, "w", newline="") as f:
                w = csv.DictWriter(f, values[0].keys())
                w.writeheader()
                w.writerows(values)
        
        # Plot the error bars
        import matplotlib.pyplot as plt
        import numpy as np

        

        # cosimulated_with_bng = [
        #     [d["position_error"] for d in simulation_vs_cosimulation_with_bng], 
        #     [d["orientation_error"] for d in simulation_vs_cosimulation_with_bng], 
        #     [d["velocity_error"] for d in simulation_vs_cosimulation_with_bng], 
        #                 ]
        # cosimulated_with_cr = [
        #     [d["position_error"] for d in simulation_vs_cosimulation_with_cr], 
        #     [d["orientation_error"] for d in simulation_vs_cosimulation_with_cr], 
        #     [d["velocity_error"] for d in simulation_vs_cosimulation_with_cr], 
        #                 ]
        # resimulated = [
        #     [d["position_error"] for d in simulation_vs_resimulation], 
        #     [d["orientation_error"] for d in simulation_vs_resimulation], 
        #     [d["velocity_error"] for d in simulation_vs_resimulation], 
        #                 ]
        def define_box_properties(plot_name, color_code, label):
            for k, v in plot_name.items():
                plt.setp(plot_name.get(k), color=color_code)
                
            # use plot function to draw a small line to name the legend.
            plt.plot([], c=color_code, label=label)
            plt.legend()

        ticks = ['Position', 'Orientation', 'Velocity']
        colors = ["#D7191C", "#61b62c", "#2C7BB6"]

        # Create a new figure
        plt.figure()

        plot_data_dict = dict()
        plots_dict = dict()

        n_bars = len(errors_dict.keys())
        center_bar = n_bars * 0.5
        width = 0.5
        offsets = np.linspace(-center_bar, +center_bar, n_bars, endpoint=True)
        
        # CENTER THE PLOTS 
        for key, color, offset in zip(errors_dict.keys(), colors, offsets):
            values = errors_dict[key]
            plot_data_dict[key] = [
                [d["position_error"] for d in values], 
                [d["orientation_error"] for d in values], 
                [d["velocity_error"] for d in values], 
            ]

            plots_dict[key] = plt.boxplot(plot_data_dict[key], 
                                          positions=np.array(np.arange(len(plot_data_dict[key])))*len(errors_dict.keys()) + offset*width,
                                          widths=width)
            
            define_box_properties(plots_dict[key], color, f"{key}")
        
        # set the x label values
        plt.xticks(np.arange(0, len(ticks) * n_bars, n_bars), ticks)
        
        # set the limit for x axis
        plt.xlim(-2, len(ticks) * n_bars)
        
        # set the limit for y axis
        # plt.ylim(0, 50)
        
        # set the title
        plt.title('Accuracy Plot')

        accuracy_file = os.path.join(plots_folder, "errors.png")
        plt.savefig(accuracy_file)
        



        