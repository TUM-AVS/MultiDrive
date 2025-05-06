# Extends the original simulation class with the BeamNG methods
import time
from copy import deepcopy

from typing import List, Dict, Any, Tuple

from omegaconf.omegaconf import OmegaConf, DictConfig

from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem as CR_PlanningProblem, PlanningProblemSet as CR_PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario as CR_Scenario
from commonroad.scenario.state import State as CR_State, InputState as CR_InputState
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.trajectory import Trajectory as CR_Trajectory

from vehiclemodels.vehicle_parameters import setup_vehicle_parameters
from vehiclemodels.vehicle_parameters import VehicleParameters as CR_VehicleParameters

from beamngpy.vehicle import Vehicle as BNG_Vehicle

from cr_beamng_cosimulation.beamng_scenario_handler import BeamNGScenarioHandler
from cr_beamng_cosimulation.controller import BngController

from cr_scenario_handler.simulation.simulation import Simulation
from cr_scenario_handler.utils.configuration import SimConfiguration, FrenetConfiguration, VehicleConfiguration
# from cr_scenario_handler.simulation.agent import AgentStatus
from cr_scenario_handler.utils.agent_status import TIMEOUT, AgentStatus

import cr_scenario_handler.utils.prediction_helpers as ph

from queue import Empty

import numpy as np


def shift_positions_from_center_to_front(state_list, distance_between_center_and_front) -> List[CR_State]:
    """
    Shifts position from rear-axle to vehicle center
    :param wb_rear_axle: distance between rear-axle and vehicle center
    """
    shifted_states = []
    # shift positions from center to front bumper 
    for state in state_list:
        orientation = state.orientation
        state_shifted = state.translate_rotate(np.array([distance_between_center_and_front * np.cos(orientation),
                                                    distance_between_center_and_front * np.sin(orientation)]), 0.0)
        shifted_states.append(state_shifted)
    return shifted_states



from shapely.geometry import Point

class BeamNGSimulation(Simulation):
    """ 
    Extends the main class for running a planner on a scenario to co-simulate the execution on 
    the BeamNG simulator
    """
    def __init__(self, config_sim, config_planner, beamng_config):
        # This will take care of preprocessing the scenario and the various planning problems
        # It will also setup the planners/agents
        super().__init__(config_sim, config_planner) 
        # BeamNG configuration holder
        self.beamng_config = beamng_config
        # BeamNG scenario handler
        self.beamng_scenario_handler = None
        # Active scenario/Current scenario
        self.bng_scenario = None
        # Logic "translating" the Frenetix Trajectories into BeamNG commands for all the vehicles
        self.bng_controller = BngController()
        
        # Unsafe variables to record colliding agents. This can be probably made safer using self.global_time_step?
        self.__colliding_agents = list()

        # Store metadata about vechiles, probably this should be already inside beamng_config or some other config!
        self.distance_between_center_and_front_dict = {}

        self.show_debug_info = False if not hasattr(config_planner, "debug") else config_planner.debug.show_debug_info

    def _cosimulate(self, time_step: int):
        #   - BeamNG controller implements the trajectories planned by the agents (if any)
        #   - Simulations moves forward one time_step
        #   - Get the vehicles' current state from BeamNG and collision data
        self.msg_logger.info(f"Cosimulating Step {time_step} self.replanning_counter = {self.replanning_counter}")

        # This is some setup activity
        # TODO is this really needed?
        margin = 0.0

        # Get the state. This causes a new connection
        curent_vehicles_state_dict, _ = self.beamng_scenario_handler.get_state_of_beamng_vehicles(-1, self.bng_scenario, use_extended_attributes=True)

        if self.config_simulation.cosimulate_with == "scriptai" and len(self.distance_between_center_and_front_dict) < len(self.agents):
            # Collect the vehicle's front positions
            for bng_vehicle_id, bng_vehicle_state in curent_vehicles_state_dict.items():
                self.msg_logger.critical(f"Collect data about Front Bumper position for agent {bng_vehicle_id}")
                self.distance_between_center_and_front_dict[bng_vehicle_id] = Point(bng_vehicle_state.position).distance(Point(bng_vehicle_state.front_position))
                self.distance_between_center_and_front_dict[bng_vehicle_id] += margin 
                # Set AI mode to manual so we can drive with the Script
                beamng_vehicle: BNG_Vehicle = self.bng_scenario.vehicles_dict[bng_vehicle_id]
                beamng_vehicle.ai.set_mode("manual")

        # Compute control commands and send them to vehicles - Collisions?
        #   4 BeamNG controller computes commands from planned trajectories and enacts the planned trajectory

        if time_step == 0:
            # skip the trajectory
            pass        
        else:
            
            for agent in self.agents:

                if agent.status == AgentStatus.COLLISION:
                    beamng_vehicle: BNG_Vehicle = self.bng_scenario.vehicles_dict[agent.id]
                    beamng_vehicle.control(steering=0.0, throttle=0.0, brake=0.0)
                    beamng_vehicle.ai_set_mode('disabled')
                    continue

                # The trajectory lastly planned
                planner_cartesian_trajectory = agent.vehicle_history[-1].prediction.trajectory

                # Visualize the Computed Trajectory in the Simulator
                self.beamng_scenario_handler.refresh_trajectory_in_simulation(agent.id, planner_cartesian_trajectory)

                beamng_vehicle: BNG_Vehicle = self.bng_scenario.vehicles_dict[agent.id]
                
                if self.replanning_counter == 0:
                    planner_cartesian_trajectory = agent.vehicle_history[-1].prediction.trajectory.state_list
                    distance_between_center_and_front = self.distance_between_center_and_front_dict[agent.id]

                    scripted_trajectory = list()

                    # Option 1: Ensure that the expected states are translated to the front bumper of the vehicle
                    adjusted_states = shift_positions_from_center_to_front(planner_cartesian_trajectory, distance_between_center_and_front)
                    actual_script_states = adjusted_states[5:6] + adjusted_states[10::2]
                    initial_time_step = adjusted_states[0].time_step
                    for s in actual_script_states:
                        trajectory_waypoint = { 
                            "x": s.position[0],
                            "y": s.position[1],
                            "z": -1.0,
                            "t": (s.time_step - initial_time_step) * self.scenario.dt 
                        }
                        scripted_trajectory.append(trajectory_waypoint)
                    
                    # Visualize the Adjusted Trajectory in the Simulator
                    bng_vehicle_state = curent_vehicles_state_dict[agent.id]
                    front_position = next(iter(shift_positions_from_center_to_front([bng_vehicle_state], distance_between_center_and_front)))
                    
                    self.beamng_scenario_handler.refresh_front_position(agent.id, front_position)
                    # The original trajectory
                    self.beamng_scenario_handler.refresh_trajectory_in_simulation(agent.id, planner_cartesian_trajectory, color="blue")
                    # The one we obtaine by translating/rotating the vehicle position to be on the front
                    # TODO Is seems overly curvy, suggesting the orientation of the vehicle in some points might be too much?
                    self.beamng_scenario_handler.refresh_trajectory_in_simulation(agent.id, adjusted_states, color="red")
                    
                    #
                    # Option 2: based on main.py. Use the planned states, but skip the first X states inside the vehicle. Also add a second delay?
                    # Cannot find a feasible solution and generally drive at lower speed?
                    # current_state = planner_cartesian_trajectory[0]
                    # actual_script_states = list()
                    # for state in planner_cartesian_trajectory:
                    #     # Skip all the states that are within the boundary of the vehicle
                    #     # TODO Ideally, this should account for the time, not just forcing 10 seconds?
                    #     if np.linalg.norm(np.array(state.position) - np.array(current_state.position)) > self.config.vehicle.length / 2.0:
                    #         trajectory_waypoint = {
                    #             "x": state.position[0],
                    #             "y": state.position[1],
                    #             "z": 0.0,
                    #             "t": (state.time_step * self.scenario.dt + 10) # -> Why +10?
                    #         }
                    #         scripted_trajectory.append(trajectory_waypoint)
                    #         actual_script_states.append(state)

                    #         # debug_id.append(beamng_scenario_handler.beamng.debug.add_spheres([(state.position[0], state.position[1])], [0.2], ["red"], cling=True, offset=3.0)[0])
                    # self.beamng_scenario_handler.refresh_front_position(agent.id, current_state)


                    # This is the actual trajectory used with Script AI
                    self.beamng_scenario_handler.refresh_trajectory_in_simulation(agent.id, actual_script_states, color="green", size_factor=3)

                    beamng_vehicle.ai.set_script(scripted_trajectory)
                else:
                    # This causes an error while reading the socket...
                    self.msg_logger.critical("Skip replanning and keep using the previous trajectory")

        #########
        # Simulations moves forward one time_step
        #########
        self.beamng_scenario_handler.step(sleep=False)

        # Get the vehicles' current state and damage information from BeamNG.
        # Each vehicle is identified by its id, which is the same (or linked) to the CommonRoad Planning Problem ID
        vehicles_cr_states: Dict[int, CR_State]
        vehicle_damages: Dict[int, bool]
        vehicles_cr_states, vehicle_damages = self.beamng_scenario_handler.get_state_of_beamng_vehicles(time_step, self.bng_scenario)

        return vehicles_cr_states, vehicle_damages
       
    def update_scenario(self, vehicles_cr_states: Dict[int, CR_State]):
        """
        updates the trajectories of agents in the global scenario but force the state observed from BeamNG
        """
        if self.global_timestep == 0:
            # no need to update initial setting
            return
        
        shape = Rectangle(self.config.vehicle.length, self.config.vehicle.width)
        agents_to_update = [agent for agent in self.agents if agent.status == AgentStatus.RUNNING]
        for agent in agents_to_update:

            if agent.status == AgentStatus.RUNNING:  # and agent.id not in colliding_agents:
                # Get the obstacle representing this agent in the CR scenario
                obs = self.scenario.obstacle_by_id(agent.id)
                # This should be the current time
                initial_timestep = obs.prediction.trajectory.initial_time_step
                # Observed State from BeamNG
                # This MUST be a RectivePlannerState object, not a StateObject
                observed_state = vehicles_cr_states[agent.id]
                
                # Harmonize the observed state from 
                # ['time_step', 'position', 'orientation', 'velocity', 'steering_angle', 'simulation_time', 'yaw_rate', 'acceleration']
                # to state:
                # ['time_step', 'position', 'orientation', 'velocity', 'acceleration', 'yaw_rate', 'slip_angle']
                # harmonized_state = deepcopy(observed_state)
                # delattr(harmonized_state, "simulation_time")

                # TODO: I am not entirely sure about this one!!!
                # Add calculated position to trajectory of dummy obstacles
                if len(obs.prediction.trajectory.state_list) > 1:
                    # Note the [2:] list slicing
                    traj = (obs.prediction.trajectory.states_in_time_interval(initial_timestep,
                                                                              self.global_timestep - 1)
                            + [observed_state]
                            + agent.vehicle_history[-1].prediction.trajectory.state_list[2:])

                else:
                    # Note the [2:] list slicing
                    traj = [observed_state] + agent.vehicle_history[-1].prediction.trajectory.state_list[2:]

                # TODO: Not sure this is correct!
                lanelet_assigned = obs.prediction.center_lanelet_assignment
                
                obs.prediction = TrajectoryPrediction(
                    Trajectory(initial_time_step=traj[0].time_step, state_list=traj), shape,
                    center_lanelet_assignment=lanelet_assigned)
        
        # TODO What's IDLE?!
        agents_to_update = [agent for agent in self.agents
                            if (agent.status != AgentStatus.RUNNING and agent.status != AgentStatus.IDLE)]
        
        # Actually update the scenario file (the world state) to be sent to all the agents for planning
        for agent in agents_to_update:
            obs = self.scenario.obstacle_by_id(agent.id)
            initial_timestep = obs.prediction.trajectory.initial_time_step  # agent.agent_state.first_timestep
            lanelet_assigned = obs.prediction.center_lanelet_assignment
            traj = obs.prediction.trajectory.states_in_time_interval(initial_timestep, agent.agent_state.last_timestep)
            obs.prediction = TrajectoryPrediction(
                Trajectory(initial_time_step=traj[0].time_step, state_list=traj), shape,
                center_lanelet_assignment=lanelet_assigned)
            
    def prestep_simulation(self):
        """
        Performs all steps necessary before a trajectory planning step:
        - collision check
        - prediction calculation
        - global scenario update
        :return: predictions, colliding agents
        """
        preproc_time = time.perf_counter()
        self.msg_logger.critical(f"Cosimulate scenario {self.scenario.scenario_id} in timestep {self.global_timestep}")
        
        vehicles_cr_states, vehicle_damages = self._cosimulate(self.global_timestep)

        # check for collisions
        colliding_agents = [k for k,v in vehicle_damages.items() if v]

        # update scenario. This update the state of all the obstacles, but the agent/agent_batch does not use this info
        # to update its current state!
        self.update_scenario(vehicles_cr_states)
        
        # Calculate new predictions
        if ((self.replanning_counter == 0 or self.config_planner.planning.replanning_frequency < 2) or
                self.config.behavior.use_behavior_planner):
            predictions = ph.get_predictions(self.config, self._predictor, self.scenario, self.global_timestep,
                                             self.prediction_horizon)
        else:
            predictions = None

        self.process_times["preprocessing"] = time.perf_counter() - preproc_time

        return predictions, colliding_agents

    def _step_parallel_simulation(self):
        """ Main function for stepping a parallel simulation.

        Computes the predictions, handles the communication with the agent batches,
        manages synchronization and termination of batches, and handles simulation-level
        logging and plotting.

        See also AgentBatch.run().

        :returns: running: True while the simulation has not completed.
        """

        # update Scenario, calculate new predictions and check for collision of prev. iteration
        self.global_predictions, colliding_agents = self.prestep_simulation()
        # send data to each batch process
        for batch in self.batch_list:
            batch.in_queue.put([self.scenario, self.global_timestep, self.global_predictions,
                                colliding_agents])

        # Plot previous timestep while batches are busy
        self.visualize_simulation(self.global_timestep - 1)

        syn_time = time.perf_counter()
        # Receive simulation step results
        for batch in reversed(self.batch_list):
            if self.event.is_set():
                # an error occured in subprocesses
                self.msg_logger.error(f"Simulation received a termination event from child processes!")
                raise ChildProcessError(f"Simulation received a termination event from child processes!")
            try:
                # update agents
                queue_dict = batch.out_queue.get(block=True, timeout=TIMEOUT)
                agent_ids = list(queue_dict.keys())

                for agent_id in agent_ids:
                    agent = self.get_agent_by_id(agent_id)
                    for attr, value in queue_dict[agent_id].items():
                        if hasattr(agent, attr):
                            if type(getattr(agent, attr)) != list:
                                setattr(agent, attr, value)
                            elif attr == "all_trajectories":
                                setattr(agent, attr, value)

                            else:
                                getattr(agent, attr).append(value)
                        else:
                            self.event.set()
                            raise AttributeError(f"{attr} is no valid attribute for the agents")
                # check if batch is finished
                [batch_finished, proc_times] = batch.out_queue.get(block=True, timeout=TIMEOUT)
                self.msg_logger.debug(f"Simulation received batch infos from batch {batch.name}")
                self.process_times[batch.name] = proc_times
                if batch_finished:
                    self.msg_logger.debug(f"Start closing Batch {batch.name}")
                    # terminate finished process
                    batch.in_queue.put("END", block=True)
                    batch.join()
                    self.batch_list.remove(batch)
                    self.msg_logger.critical(f"Closing Batch {batch.name}")
            except Empty:
                if self.event.is_set():
                    # an error occured in subprocesses
                    self.msg_logger.error(f"Simulation received a termination event from child processes!")
                    raise ChildProcessError(f"Simulation received a termination event from child processes!")
                self.msg_logger.error(f"Timeout while waiting for step results of batch {batch.name}!")
                self.event.set()
                raise Empty(f"Timeout while waiting for step results of batch {batch.name}!")
        self.process_times["time_sync"] = time.perf_counter() - syn_time

        # ALESSIO: This was missing in the original implementation
        self.replanning_counter += 1
        
        return len(self.batch_list) > 0

    def run_simulation(self):
        # Setup the BeamNG simulator and instantiate the CommonRoad scenario into it (causing the creation of BeamNG vehicles)
        # Wrapper around BeamNG
        self.beamng_scenario_handler = BeamNGScenarioHandler(self.beamng_config.host, self.beamng_config.port, 
                                                        self.beamng_config.home_folder, self.beamng_config.user_folder,
                                                        visualize_debug_spheres=self.show_debug_info)
        try:
            # Start the BeamNG simulator
            self.msg_logger.critical(f"Starting BeamNG")
            self.beamng_scenario_handler.start_simulator_if_not_running()

            # Setup the scenario in BeamNG
            # Note: At this point, self.scenario and self.planning_problem_set have been preprocessed already
            # self.config -> config_sim
            self.bng_scenario = self.beamng_scenario_handler.setup_and_start_scenario_in_beamng(self.scenario, self.planning_problem_set,
                                                                            self.config.vehicle, self.beamng_config)

            # Run the simulation. This will call methods such as prepare, post_process, and the like that we overwrite here
            super().run_simulation()

        finally:
            try:
                self.msg_logger.critical(f"Stopping BeamNG")
                self.beamng_scenario_handler.stop_simulator()
            except:
                pass
            
        