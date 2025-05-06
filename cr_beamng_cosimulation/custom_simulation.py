import abc

from typing import List, Dict


import os
import copy
import random
import time
from math import ceil
from multiprocessing import Queue, Event
from queue import Empty
from typing import List
import numpy as np
import traceback
import csv
from datetime import datetime

import psutil
from commonroad.common.util import AngleInterval
from commonroad.common.util import Interval
from commonroad.scenario.lanelet import Lanelet
from commonroad.geometry.shape import Rectangle, Polygon
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import ObstacleType, ObstacleRole, DynamicObstacle
from commonroad.scenario.state import CustomState

from commonroad.scenario.trajectory import Trajectory as CR_Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction as CR_TrajectoryPrediction

# commonroad-dc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_dc.collision.trajectory_queries import trajectory_queries

# cr-scenario-handler
import cr_scenario_handler.utils.general as general
import cr_scenario_handler.utils.multiagent_helpers as hf
from cr_scenario_handler.utils.helper_functions import find_lanelet_by_position_and_orientation
import cr_scenario_handler.utils.multiagent_logging as multi_agent_log
import cr_scenario_handler.utils.prediction_helpers as ph
import cr_scenario_handler.utils.visualization as visu
from cr_scenario_handler.utils.visualization import visualize_multiagent_scenario_at_timestep
from cr_scenario_handler.simulation.agent_batch import AgentBatch
from cr_scenario_handler.simulation.agent import Agent
from cr_scenario_handler.utils.agent_status import TIMEOUT, AgentStatus

from cr_scenario_handler.simulation.simulation import Simulation as CR_Simulation

from copy import deepcopy

class TrajectoryModifier(abc.ABC):
    
    @abc.abstractmethod
    def modify(self, original_trajectory: CR_Trajectory) -> CR_Trajectory:
        """
        Return a modified version of the original trajectory
        """
        pass

def interpolate(state_a, state_b, attribute, n_bins, position):
    value_a = getattr(state_a, attribute)
    value_b = getattr(state_b, attribute)
    # TODO Not sure this will work with 2D
    value = np.linspace(value_a, value_b, n_bins + 2, endpoint=True)[position]
    return value

class SlowdedAndAcceleratedTrajectoryModified(TrajectoryModifier):
    """
    Slow down or accelerate a given trajectory by a acceleration_factor
        use numbers smaller than 1.0 to slow down, 1.0 wont change it, above will accelerate)
    """

    def __init__(self, acceleration_factor: float):
        super().__init__()
        self.acceleration_factor = acceleration_factor

    def modify(self, original_trajectory: CR_Trajectory) -> CR_Trajectory:
        
        initial_time_step = original_trajectory.initial_time_step
        final_time_step = original_trajectory.state_list[-1].time_step

        modified_state_list = [deepcopy(s) for s in original_trajectory.state_list]

        # Alter the time, so it will take more or less to get to the next state
        for idx, state in enumerate(modified_state_list):
            state.time_step = initial_time_step + int(idx * self.acceleration_factor)
        
        # Remove states with same time_step, skip states that are too close (in time)
        filtered_state_list = list()
        time_steps = set()
        for state in modified_state_list:
            if state.time_step in time_steps:
                continue
            else:
                filtered_state_list.append(state)
                time_steps.add(state.time_step)
        
        modified_state_list = filtered_state_list
        # Interpolate states if consecutive states do not have consecutive time_steps the right time step
        filtered_state_list = list()
        time_steps = set()
        for state_before, state_after in zip(modified_state_list[:-1], modified_state_list[1:]):
            
            if len(filtered_state_list) > 0:
                # Remove the previously marked state_after
                filtered_state_list.pop()

            # Add the state
            filtered_state_list.append(state_before)
            
            # Check if we need to add one or more states in between the two states
            missing_states = state_after.time_step - state_before.time_step -1
            # Include also the last state
            for missing_time_step in range(1, missing_states + 1):
                

                new_state = deepcopy(state_before)
                new_state.time_step = state_before.time_step + missing_time_step
                
                # Interpolate all the other attributes linearly
                for attribute in [a for a in new_state.used_attributes if a not in ["time_step"]]:
                    setattr(new_state, attribute, interpolate(state_before, state_after, attribute, missing_states, missing_time_step))
                
                filtered_state_list.append(new_state)

            filtered_state_list.append(state_after)

        # Clip the trajectory at final_time_step
        modified_state_list = [s for s in filtered_state_list if s.time_step <= final_time_step]

        return CR_Trajectory(initial_time_step, modified_state_list)
    

class UnderAndOverSteeringTrajectoryModifier(TrajectoryModifier):
    """
    Slow down or accelerate a given trajectory by a acceleration_factor (use negative numbers slow down, 0 wont change it)
    """

    def __init__(self, steering_factor: float):
        super().__init__()
        self.steering_factor = steering_factor

    def modify(self, original_trajectory: CR_Trajectory) -> CR_Trajectory:
        modified_trajectory = original_trajectory
        return modified_trajectory

# dontmove
class DontMove(TrajectoryModifier):
    
    """
    Given a state make its speed, acceleration, etc. to 0.0 and keep the current position
    """

    def __init__(self):
        super().__init__()

    def modify(self, original_trajectory: CR_Trajectory) -> CR_Trajectory:
        
        initial_time_step = original_trajectory.initial_time_step
        initial_state = original_trajectory.state_at_time_step(initial_time_step)
        initial_position = initial_state.position
        initial_orientation = initial_state.orientation
        
        # Copy all the original states
        modified_state_list = [deepcopy(s) for s in original_trajectory.state_list]
        for state in modified_state_list:
            state.position = initial_position
            state.orientation = initial_orientation
            for attribute in [a for a in state.used_attributes if a not in ["time_step", "position", "orientation"]]:
                setattr(state, attribute, 0.0)

        return CR_Trajectory(initial_time_step, modified_state_list)

class CustomSimulationWithFaults(CR_Simulation):
    """
    Allows to modify the trajectory of the agents using a trajectory modifier to simulate
    the effect of missed assumptions
    """

    def __init__(self, config_sim, config_planner, trajectory_modifiers_list: List[str]):
        super().__init__(config_sim, config_planner)
        # Instantiate the trajectory_modifiers from the given list. 
        # They will be applied to ALL the agents at once
        # TODO Add better configurations later
        self.trajectory_modifiers: List[TrajectoryModifier] = list()
        for tm_name in trajectory_modifiers_list:
            if tm_name == "accelerate":
                self.trajectory_modifiers.append( SlowdedAndAcceleratedTrajectoryModified(acceleration_factor=0.2))
            elif tm_name == "decelerate":
                self.trajectory_modifiers.append( SlowdedAndAcceleratedTrajectoryModified(acceleration_factor=2.0))
            elif tm_name == "understeer":
                self.trajectory_modifiers.append( UnderAndOverSteeringTrajectoryModifier(steering_factor=-3.0))
            elif tm_name == "oversteer":
                self.trajectory_modifiers.append( UnderAndOverSteeringTrajectoryModifier(steering_factor=+1.5))
            elif tm_name == "dontmove":
                self.trajectory_modifiers.append( DontMove() )

    def prestep_simulation(self):
        """
        Performs all steps necessary before a trajectory planning step:
        - Alter the trajectories planned so far 
        - collision check with the altered trajectories
        - prediction calculation
        - global scenario update with the altered trajectories
        :return: predictions, colliding agents
        """
        preproc_time = time.perf_counter()
        
        # This is where we alter the planned trajectories
        if self.global_timestep == 0:
            return super().prestep_simulation()
        
        # In case there was NO replanning, shall we update the trajectory nevertheless?
        # if ((self.replanning_counter == 0 or self.config_planner.planning.replanning_frequency < 2) or
        #         self.config.behavior.use_behavior_planner):
        
        self.msg_logger.critical(f"Scenario {self.scenario.scenario_id} in timestep {self.global_timestep} with Injected Faults")
        altered_cartesian_trajectories = dict()
        for agent in self.agents:

                # The trajectory lastly planned
                planner_cartesian_trajectory = agent.vehicle_history[-1].prediction.trajectory
                # print(f"\n Planned trajectory from {planner_cartesian_trajectory.state_list[0].time_step} to {planner_cartesian_trajectory.state_list[-1].time_step}")

                # Take the trajectory's next state and alter it using the modifiers
                altered_cartesian_trajectory = deepcopy(planner_cartesian_trajectory)

                for trajectory_modified in self.trajectory_modifiers:
                    altered_cartesian_trajectory = trajectory_modified.modify(altered_cartesian_trajectory)

                # print(f"\n Altered trajectory from {altered_cartesian_trajectory.state_list[0].time_step} to {altered_cartesian_trajectory.state_list[-1].time_step}")

                altered_cartesian_trajectories[agent.id] = altered_cartesian_trajectory

        colliding_agents = self.custom_check_collision(altered_cartesian_trajectories)
        self.custom_update_scenario(altered_cartesian_trajectories)
        
        # Calculate new predictions
        if ((self.replanning_counter == 0 or self.config_planner.planning.replanning_frequency < 2) or
                self.config.behavior.use_behavior_planner):
            predictions = ph.get_predictions(self.config, self._predictor, self.scenario, self.global_timestep,
                                             self.prediction_horizon)
        else:
            predictions = None

        self.process_times["preprocessing"] = time.perf_counter() - preproc_time

        return predictions, colliding_agents
    

    def custom_check_collision(self, altered_cartesian_trajectories: Dict[int, CR_Trajectory]):
        ### TODO If we change the state of agents before calling this, this method can be replaced by a call to super()
        coll_objects = []
        agent_ids = []
        collided_agents = []

        # get collision objects from agents
        for agent in self.agents:

            if agent.status == AgentStatus.RUNNING:
                # Replace the last collision object with a new one based on the altered cartesian trajectory
                original_collision_object = agent.collision_objects.pop()
                time_step = original_collision_object.time_start_idx()

                altered_trajectory = altered_cartesian_trajectories[agent.id]
                altered_state = altered_trajectory.state_at_time_step(time_step)
                
                # This creates and append the replacement for the original collision object
                agent._create_collision_object(altered_state, time_step)

                altered_collision_object = agent.collision_objects[-1]
                
                # This one uses the replacement for checking collisions
                coll_objects.append(altered_collision_object)
                agent_ids.append(agent.id)

        # check if any agent collides with a static obstacle / road boundary
        coll_time_stat = trajectory_queries.trajectories_collision_static_obstacles(coll_objects, self._cc_stat)
        if any(i > -1 for i in coll_time_stat):
            index = [i for i, n in enumerate(coll_time_stat) if n != -1]
            collided_agents.extend([agent_ids[i] for i in index])

        # # check if any agent collides with a dynamic obstacle
        coll_time_dyn = trajectory_queries.trajectories_collision_dynamic_obstacles(coll_objects, self._cc_dyn)
        if any(i > -1 for i in coll_time_dyn):
            index = [i for i, n in enumerate(coll_time_dyn) if n != -1]
            collided_agents.extend([agent_ids[i] for i in index])

        # check if agents crash against each other
        if len(coll_objects) > 1:
            for index, ego in enumerate(coll_objects):
                other_agents = copy.copy(coll_objects)
                other_agents.remove(ego)
                coll_time = trajectory_queries.trajectories_collision_dynamic_obstacles([ego], other_agents,
                                                                                        method='box2d')
                if coll_time != [-1]:
                    # collision detected
                    collided_agents.append(agent_ids[index])

        if len(collided_agents) > 0:
            self.msg_logger.debug(f"Collision detected for agents {collided_agents}")
        return collided_agents

    def custom_update_scenario(self, altered_cartesian_trajectories: Dict[int, CR_Trajectory]):
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
                
                # Replace the altered states
                altered_cartesian_trajectory = altered_cartesian_trajectories[agent.id]
                observed_state = altered_cartesian_trajectory.state_at_time_step(self.global_timestep)
                # Take the remaining states
                other_state_lists = [ s for s in altered_cartesian_trajectory.state_list if s.time_step > self.global_timestep]

                # Add calculated position to trajectory of dummy obstacles
                if len(obs.prediction.trajectory.state_list) > 1:
                    # Note the [2:] list slicing
                    traj = (obs.prediction.trajectory.states_in_time_interval(initial_timestep,
                                                                              self.global_timestep - 1)
                            + [observed_state]
                            # + agent.vehicle_history[-1].prediction.trajectory.state_list[2:]
                            + other_state_lists
                            )

                else:
                    # Note the [2:] list slicing
                    traj = [observed_state] + other_state_lists
                    # agent.vehicle_history[-1].prediction.trajectory.state_list[2:]

                # TODO: Not sure this is correct!
                lanelet_assigned = obs.prediction.center_lanelet_assignment
                
                obs.prediction = CR_TrajectoryPrediction(
                    CR_Trajectory(initial_time_step=traj[0].time_step, state_list=traj), shape,
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
            obs.prediction = CR_TrajectoryPrediction(
                CR_Trajectory(initial_time_step=traj[0].time_step, state_list=traj), shape,
                center_lanelet_assignment=lanelet_assigned)