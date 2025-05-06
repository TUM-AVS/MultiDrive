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

# Allows to alter the computed trajectory, thus creating a mistmatch between planned and observed simulation states
from cr_beamng_cosimulation.custom_simulation import CustomSimulationWithFaults

#########
# UTILITIES
#########


def _load_and_check_scenario(scenario_file: str) -> Tuple[CR_Scenario, CR_PlanningProblemSet]:
    # Load the scenario and planning problems from the file
    cr_scenario: CR_Scenario
    cr_planning_problem_set: CR_PlanningProblemSet
    cr_scenario, cr_planning_problem_set = CR_FileReader(scenario_file).open()

    # Currently, we focus only on interacting AVs
    assert len(cr_scenario.obstacles) == 0, f"We cannot handle scenarios with obstacles. Currently {len(cr_scenario.obstacles)}"

    # Currently, we can handle only scenarios where AVs start from a stationary position
    for planning_problem in sorted(cr_planning_problem_set.planning_problem_dict.values(), key=lambda pprob: pprob.planning_problem_id):
        assert is_standing_state(planning_problem), f"Invalid Initial state for planning problem {planning_problem.planning_problem_id}"

    return cr_scenario, cr_planning_problem_set


def _setup_folders(logs_path: str, scenario_name: str) -> str:
    os.makedirs(logs_path, exist_ok=True)

    # dt = datetime.today()  # Get timezone naive now
    # seconds = dt.timestamp()

    # Here's where we store the logs for THIS execution
    log_path = os.path.join(logs_path, scenario_name)
    # Fail if already executed?
    os.makedirs(log_path, exist_ok=False)
    # This is the "logs" folder inside the simulation folder ... I expected it was already there...
    os.makedirs(os.path.join(log_path, "logs"), exist_ok=False)
    
    return log_path



def _build_default_configurations(mod_path: str, logs_path: str, scenario_name: str, scenario_file: str, verbose: bool) -> Tuple[Any, Any]:
    # The Frenetix Platform currently force each planner to have its own configuration
    # TODO Extend this such that each planner can have its own configuration
    # TODO Why building a new one? To avoid the same object is shared among different planners?


    # Load default values of the simulation's configurations yaml files
    config_merged = ConfigurationBuilder.build_configuration(scenario_name, path_root=mod_path, module="simulation")

    # TODO Not sure about this one...    
    config_merged.simulation["log_path"] = os.path.join(logs_path, scenario_name, config_merged.simulation.path_output)
    config_merged.simulation["mod_path"] = mod_path

    # Build the simulation configuration.
    config_sim =  SimConfiguration(config_merged)
    # This forces all the obstacles to be treated as AVs, such that their initial state and final state is
    # automatically derived from their trajectory. We do not care about that; however, this option controls also
    # how data are logged, whether multiple processes can be executed, and possibly something else ...
    config_sim.simulation.use_multiagent = True
    # Parallelize the trajectory computation. This is possible ONLY when use_multiagent is True. TODO Why?
    config_sim.simulation.multiprocessing = True
    # Ensure we store the path to the scenario file in the configuration
    config_sim.simulation.scenario_path = os.path.abspath(scenario_file)
    # Make sure we evaluate the simulation after executing it
    # We need to set this option here, since the Simulation object initializes the logger metadata
    config_sim.evaluation.evaluate_simulation = True

    # Load the default value of the planners' configuration yaml files.
    # The Frenetix Platform currently force each planner to have its own configuration
    # TODO Extend this such that each planner can have its own configuration
    # TODO Why building a new one? To avoid the same object is shared among different planners?
    config_merged = ConfigurationBuilder.build_configuration(scenario_name, path_root=mod_path, module="frenetix_motion_planner")
    config_planner = FrenetConfiguration(config_merged)

    # Force using the CPP planner - Why has this to be DEBUG?
    config_planner.debug.use_cpp = True
    config_planner.debug.show_debug_info = verbose

    return config_sim, config_planner


def _execute_the_simulation(log_path: str, simulation: CR_Simulation | BNG_Simulation | CustomSimulationWithFaults):
    evaluation = None

    try:
        # Execute the simulation
        simulation.run_simulation()
        # Evaluate the simulation
        # evaluation = evaluate_simulation(simulation)
    except Exception as e:
        error_traceback = traceback.format_exc()  # This gets the entire error traceback
        with open(os.path.join(log_path, "logs", "log_failures.csv"), 'a', newline='') as f:
            writer = csv.writer(f)
            current_time = datetime.now().strftime('%H:%M:%S')
            # Check if simulation is not None before trying to access current_timestep
            current_timestep = str(simulation.global_timestep) if simulation else "N/A"
            writer.writerow(["-- ; " + "Scenario Name: " + log_path.split("/")[-1] + " ; " +
                            "Error time: " + str(current_time) + " ; " +
                            "In Scenario Timestep: " + current_timestep + " ; " +
                            "CODE ERROR: " + str(e) + error_traceback + "\n\n\n\n"])
        raise Exception
    finally:
        try:
            # close sim_logger
            simulation.sim_logger.con.close()
            # close child processes
            simulation.close_processes()
        except:
            pass

    
    return simulation, evaluation


def _update_config_sim_with_beamng_vehicles_data(config_sim, bng_vehicle_model: str):
    """
    This one forces the dimensions of the default beamng vehicle but not the other parameters
    """
    # TODO Uniform the vehicle models in SIMULATE AND COSIMULATE

    # TODO Make sure that the size of the vehicles match those in BeamNG
    # TODO This should be the same as the simulate one!!!
    # TODO Make it possible to have a separate configuration for each planner!

    # Get the selected subset of the available models
    beamng_vehicle_models = get_beamng_vehicle_models_configuration()

    # Select the one corresponding to bng_vehicle_model
    # Default vehicle configuration use "car" aka etk800

    if bng_vehicle_model is None:
        bng_vehicle_model =  "car" # Default

    bng_vehicle_config = DictConfig({
            "beamng_vehicle_model": beamng_vehicle_models[bng_vehicle_model]["beamng_vehicle_model"],
            "length": beamng_vehicle_models[bng_vehicle_model]["length"],
            "width": beamng_vehicle_models[bng_vehicle_model]["width"]
        })

    # self.vehicle: VehicleConfiguration = VehicleConfiguration(config.vehicle)
    # vehicle_config = OmegaConf.create(default_vehicle_config)
    # initialize subclasses automatically
    # config_sim.vehicle = vehicle_config
    for k, v in bng_vehicle_config.items():
        setattr(config_sim.vehicle, k, v)


def _store_configurations(output_folder: str, config_sim, config_planner):
    # The confs are NOT Omega conf?
    output_config_sim = os.path.join(output_folder, "config_sim.pickle")
    with open(output_config_sim, 'wb') as handle:
        pickle.dump(config_sim, handle, protocol=pickle.HIGHEST_PROTOCOL)
    
    output_config_planner = os.path.join(output_folder, "config_planner.pickle")
    with open(output_config_planner, 'wb') as handle:
        pickle.dump(config_planner, handle, protocol=pickle.HIGHEST_PROTOCOL)


def _store_scenario(output_folder: str, scenario: CR_Scenario, planning_problem_set: CR_PlanningProblemSet, file_name="scenario.xml"):
    output_scenario_file = os.path.join(output_folder, file_name)

    author = scenario.author
    affiliation = scenario.affiliation
    source = scenario.source
    tags = scenario.tags
    location = scenario.location

    CR_FileWriter(scenario, 
        planning_problem_set, 
        author,
        affiliation,
        source,
        tags,
        location, decimal_precision=20).write_to_file(output_scenario_file)


def _store_executed_scenario(output_folder: str, executed_scenario: CR_Scenario, file_name="executed_scenario.xml"):
    output_scenario_file = os.path.join(output_folder, file_name)
    CR_FileWriter(executed_scenario, 
        None, # planning_problem_set, 
        decimal_precision=20).write_scenario_to_file(output_scenario_file)


@click.command()
@click.pass_context
@click.option('--scenario-file', type=click.Path(exists=True))
@click.option('--vehicle-model', 'bng_vehicle_model', type=click.Choice(["car", "mini", "jeep"], case_sensitive=False), default="car")
def simulate(ctx, scenario_file: str, bng_vehicle_model: str):
    """
    Simulate the CommonRoad scenario using CommonRoad
    """

    verbose = ctx.obj["verbose"]

    original_scenario, original_planning_problem_set = _load_and_check_scenario(scenario_file)
    
    # scenario_name defines the name of the folder inside logs_path that will contain the results
    scenario_name = ctx.obj["scenario_name"]
    
    # Default location for storing ALL the simulation results
    output_folder = ctx.obj['output_folder']
    simulation_output_folder = _setup_folders(output_folder, scenario_name)

    mod_path = FRENETIX_MODULE_PATH
    config_sim, config_planner = _build_default_configurations(mod_path, output_folder, scenario_name, scenario_file, verbose)
    _update_config_sim_with_beamng_vehicles_data(config_sim, bng_vehicle_model)
        
    # TODO Is this necessary? Maybe to remember in the output folder that this scenario was simulated?
    config_sim.simulation.cosimulate = False

    config_sim.visualization.save_plots = verbose

    # Use Base CR Simulation
    simulation = CR_Simulation(config_sim, config_planner)
    
    # Execute the simulation and return whatever result has to be returned
    simulation, evaluation = _execute_the_simulation(simulation_output_folder, simulation)

    executed_scenario = simulation.scenario

    # Copy all the configurations and the input scenarios into the output folder
    _store_configurations(output_folder, config_sim, config_planner)
    _store_scenario(output_folder, original_scenario, original_planning_problem_set, file_name="scenario_to_simulate.xml")
    _store_executed_scenario(output_folder, executed_scenario, file_name="simulated_scenario.xml")

    # Register in the context where to store information about the execution time
    ctx.obj["execution_time_file"] = os.path.join(output_folder, "simulation_time.txt")

    return simulation, evaluation

@click.command()
@click.pass_context
@click.option('--scenario-file', type=click.Path(exists=True))
@click.option('--vehicle-model', 'bng_vehicle_model', type=click.Choice(["car", "mini", "jeep"], case_sensitive=False), default="car")
def cosimulate_with_beamng(ctx, scenario_file: str, bng_vehicle_model:str):
    """
    Simulate the CommonRoad scenario using BeamNG.tech and its native controller ScritpAI
    """

    verbose = ctx.obj["verbose"]

    original_scenario, original_planning_problem_set = _load_and_check_scenario(scenario_file)

    scenario_name = ctx.obj["scenario_name"]
    
    # Default location for storing ALL the simulation results
    output_folder = ctx.obj['output_folder']
    simulation_output_folder = _setup_folders(output_folder, scenario_name)

    # Location where the Frenetix configurations are stored
    mod_path = FRENETIX_MODULE_PATH
    config_sim, config_planner = _build_default_configurations(mod_path, output_folder, scenario_name, scenario_file, verbose)
    _update_config_sim_with_beamng_vehicles_data(config_sim, bng_vehicle_model)

    # TODO Is this necessary? Maybe to remember in the output folder that this scenario was co-simulated?
    config_sim.simulation.cosimulate = True
    config_sim.simulation.cosimulate_with = "scriptai"
    config_sim.visualization.save_plots = verbose
    #
    # NOTE: With parallel execution there's an issue with propagating the agent states to "remote" agents,  
    #   so we forcefully use the sequential simulation here.
    config_sim.simulation.multiprocessing = False
    # Force replanning at each cycle if needed
    # config_planner.planning.replanning_frequency = 1

    # Load the additional (default) configurations from the beamng.ini file
    # TODO Consider adding beamng configuration overrides
    beamng_config = get_beamng_configuration()

    # Since we are co-simulating CommonRoad scenarios using BeamNG we need a different simulation class
    simulation = BNG_Simulation(config_sim, config_planner, beamng_config)

    # Execute the simulation and return whatever result has to be returned
    simulation, evaluation = _execute_the_simulation(simulation_output_folder, simulation)

    executed_scenario = simulation.scenario

    # Copy all the configurations and the input scenarios into the output folder
    _store_configurations(output_folder, config_sim, config_planner)
    _store_scenario(output_folder, original_scenario, original_planning_problem_set, file_name="scenario_to_cosimulate_with_bng.xml")
    _store_executed_scenario(output_folder, executed_scenario, file_name="cosimulated_scenario_with_bng.xml")

    # Register in the context where to store information about the execution time
    ctx.obj["execution_time_file"] = os.path.join(output_folder, "cosimulation_with_bng_time.txt")

    return simulation, evaluation


def _create_planning_problems_for_agent_obstacles(scenario: CR_Scenario):
    import numpy as np
    from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
    from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
    from commonroad.scenario.state import InitialState, CustomState
    from commonroad.planning.goal import GoalRegion, Interval
    from commonroad.geometry.shape import Polygon
     
    planning_problem_list = []
    for obstacle in scenario.obstacles:
        # define initial state for planning problem
        initial_state = obstacle.initial_state
        if not hasattr(initial_state, 'acceleration'):
            initial_state.acceleration = 0.
        # define goal state for planning problem

        final_state = obstacle.prediction.trajectory.final_state
        lanelet_id = find_lanelet_by_position_and_orientation(scenario.lanelet_network, final_state.position,
                                                                final_state.orientation)
        lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0])

        # to increase goal area (keep non-original planning-problem vehicles
        # as long as possible without loss of scenario success),
        # try to add successor lanelet as goal area as well
        if len(lanelet.successor) > 0:
            lanelet = Lanelet.merge_lanelets(lanelet, scenario.lanelet_network.find_lanelet_by_id(
                lanelet.successor[0]))

        # get index of closest center_vortex
        index_vortex = np.argmin(np.linalg.norm(lanelet.center_vertices - final_state.position, axis=1))

        # get curvilinear distance to final position
        if (np.linalg.norm(lanelet.center_vertices[index_vortex] - lanelet.center_vertices[0]) >
                np.linalg.norm(final_state.position - lanelet.center_vertices[0])):
            dist = lanelet._compute_polyline_cumsum_dist(
                [np.vstack((lanelet.center_vertices[:index_vortex], final_state.position
                            ))])[-1]

        else:
            dist = lanelet._compute_polyline_cumsum_dist([np.vstack((lanelet.center_vertices[:index_vortex + 1],
                                                                        final_state.position))])[-1]

        # get final position of obstacle as initial point of goal area
        (final_center, final_right, final_left, index) = lanelet.interpolate_position(dist)

        # goal area is defined as current lanelet from final_state.position - safety buffer of 0.5m
        # to end of lanelet + subsequent lanelet
        if (np.linalg.norm(lanelet.center_vertices[index_vortex] - initial_state.position)
                >= np.linalg.norm(final_center - initial_state.position) and
                (np.linalg.norm(lanelet.center_vertices[index] - initial_state.position)
                    <= np.linalg.norm(final_center - initial_state.position))):
            left = np.vstack((final_left,
                                lanelet.left_vertices[index_vortex:]))
            right = np.vstack((final_right,
                                lanelet.right_vertices[index_vortex:]))
            while np.linalg.norm(left[-1] - left[0]) <= 3 or np.linalg.norm(right[-1] - right[0]) <= 3:
                # extend goal area to front if it is too small
                index_vortex -= 1
                left = np.vstack((lanelet.left_vertices[index_vortex], left))
                right = np.vstack((lanelet.right_vertices[index_vortex], right))

        elif (np.linalg.norm(lanelet.center_vertices[index_vortex] - initial_state.position)
                >= np.linalg.norm(final_center - initial_state.position) and
                (np.linalg.norm(lanelet.center_vertices[index] - initial_state.position)
                > np.linalg.norm(final_center - initial_state.position))):

            left = np.vstack(
                (lanelet.left_vertices[:index_vortex + 1], final_left))  # lanelet.left_vertices[index+1:]
            right = np.vstack((lanelet.right_vertices[:index_vortex + 1], final_right))

            while np.linalg.norm(left[-1] - left[0]) <= 3 or np.linalg.norm(right[-1] - right[0]) <= 3:
                # extend goal area to front if it is too small
                index_vortex += 1
                left = np.vstack((left, lanelet.left_vertices[index_vortex]))
                right = np.vstack((right, lanelet.right_vertices[index_vortex]))

        elif (np.linalg.norm(lanelet.center_vertices[index_vortex] - initial_state.position)
                <= np.linalg.norm(final_center - initial_state.position) and
                (np.linalg.norm(lanelet.center_vertices[index] - initial_state.position)
                <= np.linalg.norm(final_center - initial_state.position))):
            left = np.vstack((final_left,
                                lanelet.left_vertices[index + 1:]))  # lanelet.left_vertices[index+1:]
            right = np.vstack((final_right,
                                lanelet.right_vertices[index + 1:]))

            while np.linalg.norm(left[-1] - left[0]) <= 3 or np.linalg.norm(right[-1] - right[0]) <= 3:
                # extend goal area to front if it is too small
                index -= 1
                left = np.vstack((lanelet.left_vertices[index], left))
                right = np.vstack((lanelet.right_vertices[index], right))
        else:
            left = np.vstack(
                (final_left, lanelet.left_vertices[index_vortex:]))  # lanelet.left_vertices[index+1:]
            right = np.vstack(
                (final_right, lanelet.right_vertices[index_vortex:]))  # lanelet.right_vertices[index+1:]

            while np.linalg.norm(left[-1] - left[0]) <= 3 or np.linalg.norm(right[-1] - right[0]) <= 3:
                # extend goal area to front if it is too small
                index_vortex -= 1
                left = np.vstack((lanelet.left_vertices[index_vortex], left))
                right = np.vstack((lanelet.rigth_vertices[index_vortex], right))

        position = np.concatenate((left, right[::-1]), axis=0)

        goal_state = CustomState(
            time_step=Interval(final_state.time_step - 20, final_state.time_step + 20),
                                position=Polygon(position),  # Circle(1.5, final_state.position),
                                    velocity=Interval(final_state.velocity - 2, final_state.velocity + 2)
                                )
        # create planning problem
        problem = PlanningProblem(obstacle.obstacle_id, initial_state,
                                    GoalRegion(list([goal_state]), lanelets_of_goal_position={0: lanelet_id}))
        planning_problem_list.append(problem)

    return planning_problem_list