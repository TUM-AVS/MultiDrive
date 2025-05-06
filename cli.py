# This file is organized following the example described here: https://stackoverflow.com/questions/34643620/how-can-i-split-my-click-commands-each-with-a-set-of-sub-commands-into-multipl

import click

from cr_beamng_cosimulation import commands as simulation_commands
from cr_beamng_cosimulation import analysis_commands as analysis_commands

######## Monkey Patching problematic libraries

def apply_monkey_path():
    # This works because we patch the ConfigurationBuilder that we import here!
    from cr_scenario_handler.utils.configuration_builder import ConfigurationBuilder

    def patched_build_configuration(scenario_name: str, path_root: str = None, dir_config="configurations", module="simulation"):
        # Do not read configurations from CLI
        if path_root is None:
            path_root = os.path.normpath(os.path.join(os.path.dirname(__file__), "../../"))
        ConfigurationBuilder.set_paths(path_root=path_root, dir_config=dir_config, dir_config_default=module)
        config_default = ConfigurationBuilder.construct_configuration(scenario_name)
        return config_default

    ConfigurationBuilder._original_build_configuration = ConfigurationBuilder.build_configuration
    ConfigurationBuilder.build_configuration = patched_build_configuration


    import os
    from typing import List, Union, Dict
    from PIL import Image

    import matplotlib
    from matplotlib import pyplot as plt
    import imageio.v3 as iio
    import numpy as np
    import matplotlib.cm as cm
    import matplotlib.colors as colors
    from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
    from commonroad.scenario.state import State, CustomState
    from commonroad.planning.planning_problem import PlanningProblem
    from cr_scenario_handler.utils.agent_status import AgentStatus
    # commonroad_dc
    from commonroad_dc import pycrcc

    from commonroad.scenario.scenario import Scenario
    from commonroad.visualization.draw_params import MPDrawParams, DynamicObstacleParams, ShapeParams
    from commonroad.visualization.mp_renderer import MPRenderer
    from commonroad.geometry.shape import Rectangle

    from cr_scenario_handler.utils.configuration import Configuration

    from wale_net_lite.visualization import draw_uncertain_predictions

    from cr_scenario_handler.utils.visualization import green_to_red_colormap, colors_spec, lightcolors, darkcolors

    # Same as the original but store images always as PNG
    def patched_visualize_agent_at_timestep(scenario: Scenario, planning_problem: PlanningProblem, ego: DynamicObstacle,
                                timestep: int, config, log_path: str, traj_set=None, optimal_traj=None,
                                ref_path: np.ndarray = None, rnd: MPRenderer = None, predictions: dict = None,
                                plot_window: int = None, visible_area=None, occlusion_map=None,
                                behavior_module_state: dict = None, save: bool = False, show: bool = False,
                                gif: bool = False, replanning_counter: int = 0):
        
        # Only create renderer if not passed in
        if rnd is None:
            # Assuming ego.prediction.trajectory.state_list[0].position returns a constant value
            ego_start_pos = ego.prediction.trajectory.state_list[0].position
            if plot_window > 0:
                plot_limits = [-plot_window + ego_start_pos[0], plot_window + ego_start_pos[0],
                            -plot_window + ego_start_pos[1], plot_window + ego_start_pos[1]]
                rnd = MPRenderer(plot_limits=plot_limits, figsize=(10, 10))
            else:
                rnd = MPRenderer(figsize=(20, 10))

        # set ego vehicle draw params
        ego_params = DynamicObstacleParams()
        ego_params.time_begin = timestep
        ego_params.draw_icon = config.visualization.draw_icons
        ego_params.show_label = True
        ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
        ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
        ego_params.vehicle_shape.occupancy.shape.zorder = 50
        ego_params.vehicle_shape.occupancy.shape.opacity = 1

        obs_params = MPDrawParams()
        obs_params.dynamic_obstacle.time_begin = timestep
        obs_params.dynamic_obstacle.draw_icon = config.visualization.draw_icons
        obs_params.dynamic_obstacle.show_label = True
        obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
        obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#003359"

        obs_params.static_obstacle.show_label = True
        obs_params.static_obstacle.occupancy.shape.facecolor = "#a30000"
        obs_params.static_obstacle.occupancy.shape.edgecolor = "#756f61"

        # visualize scenario, planning problem, ego vehicle
        scenario.draw(rnd, draw_params=obs_params)
        planning_problem.draw(rnd)
        ego.draw(rnd, draw_params=ego_params)

        rnd.render()

        # visualize optimal trajectory
        if optimal_traj:
            optimal_traj_positions = np.array([(state.position[0], state.position[1]) for state in optimal_traj.state_list[0+replanning_counter:]])
            rnd.ax.plot(optimal_traj_positions[:, 0], optimal_traj_positions[:, 1], 'kx-', markersize=1.5, zorder=21, linewidth=2.0)

        # draw visible sensor area
        if visible_area is not None:
            if visible_area.geom_type == "MultiPolygon":
                for geom in visible_area.geoms:
                    rnd.ax.fill(*geom.exterior.xy, "g", alpha=0.2, zorder=10)
            elif visible_area.geom_type == "Polygon":
                rnd.ax.fill(*visible_area.exterior.xy, "g", alpha=0.2, zorder=10)
            else:
                for obj in visible_area.geoms:
                    if obj.geom_type == "Polygon":
                        rnd.ax.fill(*obj.exterior.xy, "g", alpha=0.2, zorder=10)

        # draw occlusion map - first version
        if occlusion_map is not None:
            cmap = colors.LinearSegmentedColormap.from_list('rg', ["r", "y", "g"], N=10)
            scatter = rnd.ax.scatter(occlusion_map[:, 1], occlusion_map[:, 2], c=occlusion_map[:, 4], cmap=cmap, zorder=25, s=5)
            handles, labels = scatter.legend_elements(prop="colors", alpha=0.6)
            rnd.ax.legend(handles, labels, loc="upper right", title="Occlusion")

        # visualize sampled trajectory bundle
        if traj_set is not None and replanning_counter == 0:
            valid_traj = [obj for obj in traj_set if obj.valid is True and obj.feasible is True]
            invalid_traj = [obj for obj in traj_set if obj.valid is False or obj.feasible is False]
            norm = matplotlib.colors.Normalize(
                vmin=0,
                vmax=len(valid_traj),
                clip=True,
            )
            mapper = cm.ScalarMappable(norm=norm, cmap=green_to_red_colormap())
            step = int(len(invalid_traj) / 50) if int(len(invalid_traj) / 50) > 2 else 1
            for idx, val in enumerate(reversed(valid_traj)):
                if not val._coll_detected:
                    color = mapper.to_rgba(len(valid_traj) - 1 - idx)
                    plt.plot(val.cartesian.x, val.cartesian.y,
                            color=color, zorder=20, linewidth=1.0, alpha=1.0, picker=False)
                else:
                    plt.plot(val.cartesian.x, val.cartesian.y,
                            color='cyan', zorder=20, linewidth=1.0, alpha=0.8, picker=False)
            for ival in range(0, len(invalid_traj), step):
                plt.plot(invalid_traj[ival].cartesian.x, invalid_traj[ival].cartesian.y,
                        color="#808080", zorder=19, linewidth=0.8, alpha=0.4, picker=False)

        # visualize predictions
        if predictions is not None:
            predictions_copy = predictions
            if ego.obstacle_id in predictions.keys():
                predictions_copy = predictions.copy()
                # Remove the entry with key 60000 from the copy
                predictions_copy.pop(ego.obstacle_id, None)
            if config.prediction.mode == "walenet":
                draw_uncertain_predictions(predictions_copy, rnd.ax)

        # visualize reference path
        if ref_path is not None:
            rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                        label='reference path')

        # visualize behavior states
        if behavior_module_state is not None:
            if config.behavior.use_behavior_planner and config.behavior.visualize_states:
                rnd.f.texts = []  # remove the texts, otherwise they are stacking
                behavior_text_left = ""
                if config.behavior.visualization_mode == "SLIM":
                    behavior_text_left = (
                        f"behavior_state_static: {str(behavior_module_state.get('behavior_state_static'))}\n"
                        f"behavior_state_dynamic: {str(behavior_module_state.get('behavior_state_dynamic'))}\n"
                    )
                if config.behavior.visualization_mode in ["BASIC", "EXTENDED", "FULL"]:
                    behavior_text_left = (
                        f"street_setting: {str(behavior_module_state.get('street_setting'))}\n"
                        f"behavior_state_static: {str(behavior_module_state.get('behavior_state_static'))}\n"
                        f"situation_state_static: {str(behavior_module_state.get('situation_state_static'))}\n"
                        f"behavior_state_dynamic: {str(behavior_module_state.get('behavior_state_dynamic'))}\n"
                        f"situation_state_dynamic: {str(behavior_module_state.get('situation_state_dynamic'))}\n"
                    )
                if config.behavior.visualization_mode in ["EXTENDED", "FULL"]:
                    behavior_text_left += (
                        f"goal_velocity: {behavior_module_state.get('goal_velocity'):.2f}\n"
                        f"desired_velocity: {behavior_module_state.get('desired_velocity'):.2f}\n"
                        f"velocity: {behavior_module_state.get('velocity'):.2f}\n"
                    )
                if config.behavior.visualization_mode in ["FULL"]:
                    if behavior_module_state.get('TTC') is None:
                        ttc_str = f"None"
                    else:
                        ttc_str = f"{behavior_module_state.get('TTC'):.2f}"
                    behavior_text_left += (
                        f"TTC: {ttc_str}\n"
                        f"MAX: {behavior_module_state.get('MAX'):.2f}\n"
                        f"stop_point_dist: {behavior_module_state.get('stop_point_dist'):.2f}\n"
                        f"desired_velocity_stop_point: {behavior_module_state.get('desired_velocity_stop_point'):.2f}\n"
                        f"stop_point_mode: {str(behavior_module_state.get('stop_point_mode'))}\n"
                        f"lane_change_target_lanelet_id: {str(behavior_module_state.get('lane_change_target_lanelet_id'))}\n"
                        f"slowing_car_for_traffic_light: {str(behavior_module_state.get('slowing_car_for_traffic_light'))}\n"
                        f"waiting_for_green_light: {str(behavior_module_state.get('waiting_for_green_light'))}\n"
                        f"condition_factor: {behavior_module_state.get('condition_factor'):.2f}\n"
                        f"lon_dyn_cond_factor: {behavior_module_state.get('lon_dyn_cond_factor'):.2f}\n"
                        f"lat_dyn_cond_factor: {behavior_module_state.get('lat_dyn_cond_factor'):.2f}\n"
                        f"visual_cond_factor: {behavior_module_state.get('visual_cond_factor'):.2f}\n"
                    )

                rnd.f.text(0.15, 0.85, behavior_text_left, ha='left', va='top', fontsize=12)

        # save as .png file
        if config.visualization.save_plots or save or gif:
            plot_dir = os.path.join(log_path, "plots")
            os.makedirs(plot_dir, exist_ok=True)
            if config.visualization.save_gif or gif:
                plt.axis('off')
                plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.png", format='png', dpi=300, bbox_inches='tight', pad_inches=0)
            else:
                # plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.svg", format='svg')
                plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.png", format='png')

        # show plot
        if config.visualization.show_plots or show:
            matplotlib.use("TkAgg")
            plt.pause(0.0001)

    def patched_visualize_multiagent_scenario_at_timestep(scenario: Scenario,
                                              agent_list: List,
                                              timestep: int,
                                              config: Configuration, log_path: str,
                                              orig_pp_id: List=None,
                                              predictions: dict = None,
                                              rnd: MPRenderer = None,
                                              plot_window: int = None,
                                              save: bool = False, show: bool = False, gif: bool = False):
        # create renderer object (if no existing renderer is passed)
        if rnd is None:
            rnd = MPRenderer(figsize=(20, 10))

            if plot_window is not None:
                # focus on window around all agents
                left = np.inf
                right = - np.inf
                top = - np.inf
                bottom = np.inf
                for agent in agent_list:
                    try:
                        ini_pos = agent.planning_problem.initial_state.position
                        if hasattr(agent.planning_problem.goal.state_list[0].position, "shapes"):
                            goal_pos = agent.planning_problem.goal.state_list[0].position.shapes[0].center
                        elif hasattr(agent.planning_problem.goal.state_list[0].position, "center"):
                            goal_pos = agent.planning_problem.goal.state_list[0].position.center
                        # else:
                        #     raise ValueError

                        left = min(left, goal_pos[0], ini_pos[0])
                        right = max(right, goal_pos[0], ini_pos[0])
                        top = max(top, goal_pos[1], ini_pos[1])
                        bottom = min(bottom, goal_pos[1], ini_pos[1])
                    except AttributeError:
                        pass

                rnd.plot_limits = [-plot_window + left,
                                plot_window + right,
                                -plot_window + bottom,
                                plot_window + top]

        # Set obstacle parameters
        obs_params = MPDrawParams()
        obs_params.dynamic_obstacle.time_begin = timestep
        obs_params.dynamic_obstacle.draw_icon = config.visualization.draw_icons
        obs_params.dynamic_obstacle.show_label = config.visualization.show_labels
        obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
        obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#003359"

        obs_params.static_obstacle.show_label = config.visualization.show_labels
        obs_params.static_obstacle.occupancy.shape.facecolor = "#A30000"
        obs_params.static_obstacle.occupancy.shape.edgecolor = "#756F61"

        # visualize scenario
        scenario.draw(rnd, draw_params=obs_params)

        # Visualize agents and planning problems
        # for i in range(len(agent_list)):
        for agent in agent_list:
            if agent.status != AgentStatus.RUNNING:
                continue
            # set ego vehicle draw params
            ego_params = DynamicObstacleParams()
            ego_params.time_begin = timestep
            ego_params.draw_icon = config.visualization.draw_icons
            ego_params.show_label = config.visualization.show_labels

            # Use standard colors for single-agent plots
            if len(agent_list) == 1:
                ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
                ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
            elif orig_pp_id:
                if agent.id in orig_pp_id:
                    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
                    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
                else:
                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
                    obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#003359"
            else:
                ego_params.vehicle_shape.occupancy.shape.facecolor = \
                    colors_spec[agent.id % len(colors_spec)]
                ego_params.vehicle_shape.occupancy.shape.edgecolor = \
                    darkcolors[agent.id % len(darkcolors)]
            ego_params.vehicle_shape.occupancy.shape.zorder = 50
            ego_params.vehicle_shape.occupancy.shape.opacity = 1

            # Visualize planning problem and agent
            # if multiple_agents:
            #     planning_problems.find_planning_problem_by_id(agent.obstacle_id).draw(rnd)
            # else:

            agent.vehicle_history[-1].draw(rnd, draw_params=ego_params)
            agent.planning_problem.draw(rnd)
        rnd.render()
        for agent in agent_list:
            if agent.status != AgentStatus.RUNNING:
                continue
            # visualize optimal trajectory
            history = [i for i in agent.vehicle_history if i.initial_state.time_step == timestep]
            if history:
                history = history[0]

                if history.prediction is not None and config.visualization.draw_calc_traj:
                    rnd.ax.plot([i.position[0] for i in history.prediction.trajectory.state_list],
                                [i.position[1] for i in history.prediction.trajectory.state_list],
                                color='k' , marker='x', markersize=1.5, zorder=21, linewidth=2.0, label='optimal trajectory')

            if agent.all_trajectories is not None and agent.config_planner.debug.draw_traj_set:
                valid_traj = [obj for obj in agent.all_trajectories if obj.valid is True and obj.feasible is True]
                invalid_traj = [obj for obj in agent.all_trajectories if obj.valid is False or obj.feasible is False]
                norm = matplotlib.colors.Normalize(
                    vmin=0,
                    vmax=len(valid_traj),
                    clip=True,
                )
                mapper = cm.ScalarMappable(norm=norm, cmap=green_to_red_colormap())
                # step_inv = int(len(invalid_traj) / 50) if int(len(invalid_traj) / 50) > 2 else 1
                step_val = int(len(valid_traj)/50)  if int(len(valid_traj) / 50) > 2 else 1
                step_inv=step_val
                for idx in range(0, len(valid_traj), step_val):
                    val = valid_traj[idx]
                    if not val._coll_detected:
                        color = mapper.to_rgba(idx)
                        color = lightcolors[idx % len(lightcolors)]
                        plt.plot(val.cartesian.x, val.cartesian.y,
                                color=color, zorder=20, linewidth=1.0, alpha=1.0, picker=False)
                    else:
                        plt.plot(val.cartesian.x, val.cartesian.y,
                                color='cyan', zorder=20, linewidth=1.0, alpha=0.8, picker=False)
                for ival in range(0, len(invalid_traj), step_inv):
                    plt.plot(invalid_traj[ival].cartesian.x, invalid_traj[ival].cartesian.y,
                            color="#808080", zorder=19, linewidth=0.8, alpha=0.4, picker=False)

            if agent.reference_path is not None and config.visualization.draw_reference_path:

                rnd.ax.plot(agent.reference_path[:, 0], agent.reference_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                            label='reference path')

        # visualize predictions
        if predictions is not None and config.visualization.draw_predictions:
            draw_uncertain_predictions(predictions, rnd.ax)

        if save or gif:
            plot_dir = os.path.join(log_path, "plots")
            os.makedirs(plot_dir, exist_ok=True)
            if gif:
                plt.axis('off')
                plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.png", format='png', dpi=300, bbox_inches='tight',
                            pad_inches=0)
            else:
                # plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.svg", format='svg')
                plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.png", format='png')

        if show:
            matplotlib.use("TkAgg")
            plt.pause(0.0001)


    # This does not work because we need to patch the visualization imported in the module using it!
    # from cr_scenario_handler.utils import visualization
    # We need to first get an handle to the simulation package
    
    from cr_scenario_handler.simulation import simulation
    # Then we need to update "there" the visualization import
    visualize_multiagent_scenario_at_timestep = getattr(simulation, "visualize_multiagent_scenario_at_timestep")
    # Backup the original function
    setattr(simulation, "_original_visualize_multiagent_scenario_at_timestep", visualize_multiagent_scenario_at_timestep)
    # Overwrite the new function
    setattr(simulation, "visualize_multiagent_scenario_at_timestep", patched_visualize_multiagent_scenario_at_timestep)

    ########################################################################################################################
    # Patch the agent class accessed in the simulation module
    ########################################################################################################################

    from cr_scenario_handler.simulation import agent
    visualize_agent_at_timestep = getattr(agent, "visualize_agent_at_timestep")
    setattr(agent, "_original_visualize_agent_at_timestep", visualize_agent_at_timestep)
    setattr(agent, "visualize_agent_at_timestep", patched_visualize_agent_at_timestep)

    import csv
    import json
    import logging
    import os
    import sqlite3
    import sys

    from omegaconf import DictConfig, ListConfig

    from cr_scenario_handler.utils.agent_status import AgentStatus, TIMEOUT
    from cr_scenario_handler.utils.configuration import Configuration

    from rich.logging import RichHandler

    def patched__init__(self, config):
        self.config = config
        self.eval_conf = config.evaluation
        self.scenario = self.config.simulation.name_scenario
        self.original_planning_problem_id = None
        self.log_path = self.config.simulation.log_path
        # self.log_path = log_path if self.scenario not in log_path else log_path.replace(self.scenario, "")
        self.log_time = self.eval_conf.evaluate_runtime
        self.scenario = self.config.simulation.name_scenario

        os.makedirs(self.log_path, exist_ok=True)
        if os.path.exists(os.path.join(self.log_path, "simulation.db")) and not config.evaluation.evaluate_simulation:
            os.remove(os.path.join(self.log_path, "simulation.db"))

        self.con = sqlite3.connect(os.path.join(self.log_path, "simulation.db"), timeout=TIMEOUT,
                                                isolation_level="EXCLUSIVE")

        self.con.executescript("""
            PRAGMA journal_mode = OFF;
            PRAGMA temp_store = MEMORY;
        """)
        self.con.commit()

        self.create_tables()

    # from cr_scenario_handler.utils.multiagent_logging import SimulationLogger
    # from cr_scenario_handler.simulation import simulation
    multi_agent_log = getattr(simulation, "multi_agent_log")
    multi_agent_log.SimulationLogger._original__init__ = multi_agent_log.SimulationLogger.__init__
    multi_agent_log.SimulationLogger.__init__ = patched__init__

    def patched_update_batch(self):
        for agent in reversed(self.running_agent_list):
            if agent.status > AgentStatus.RUNNING:
                self.terminated_agent_list.append(agent)
                self.running_agent_list.remove(agent)
                # with (open(os.path.join(self.mod_path, "logs", "score_overview.csv"), 'a') as file):
                # log_path contains already the logs folder ?!
                with (open(os.path.join(self.log_path, "score_overview.csv"), 'a') as file):
                    msg = "Success" if agent.status == AgentStatus.COMPLETED_SUCCESS else "Failed"
                    line = str(agent.scenario.scenario_id) + ";" + str(agent.id) + ";" + str(agent.current_timestep) + ";" + \
                           str(agent.status) + ";" + str(agent.agent_state.message) + ";" + msg + "\n"
                    file.write(line)

            self.out_queue_dict[agent.id] = {"agent_state": agent.agent_state,
                                             "collision_objects": agent.collision_objects[-1],
                                             "vehicle_history": agent.vehicle_history[-1],
                                             "record_state_list": agent.record_state_list[-1],
                                             "record_input_list": agent.record_input_list[-1],
                                             "planning_times": agent.planning_times[-1],
                                             }
            
    # from cr_scenario_handler.simulation.agent_batch import AgentBatch
    agent_batch_cls = getattr(simulation, "AgentBatch")
    agent_batch_cls._original_update_batch = agent_batch_cls._update_batch
    agent_batch_cls._update_batch = patched_update_batch

    ####################################################################################################
    # Force the Agent current state (x_0 + x_cl) from the shared scenario. Note that Agent instances
    #   are passed to the AgentBatch by the Simulation class
    ####################################################################################################
    
    from commonroad.scenario.state import InitialState
    from frenetix_motion_planner.state import ReactivePlannerState
    
    from copy import deepcopy

    import cr_scenario_handler.utils.prediction_helpers as ph
    import cr_scenario_handler.utils.multiagent_helpers as hf

    def patched_update_agent(self, scenario: Scenario, time_step: int, global_predictions: dict,
                     collision: bool = False):
        """ 
        Update the scenario to synchronize the agents, including the current state of the agent.
        This makes agent synchronization kinda bi-directional, TODO Fix the predictions and the state
        of the agent that has all the history/states broken?

        :param scenario:
        :param time_step:
        :param global_predictions:
        :param collision:
        """
        self.agent_state.log_running(time_step)
        
        # Get the state of the agent from the scenario
        ego_as_obstacle = scenario.obstacle_by_id(self.id)
        ego_current_state = ego_as_obstacle.state_at_time(time_step)
        ego_state_as_initial_state = ego_current_state.convert_state_to_state(InitialState())
        
        # Set initial state and curvilinear state
        # This one shift the position of the vehicle to the rear axle... but is it correct?
        # NOTE: This one changes the object: ego_state_as_initial_state that's why we have a deepcopy
        updated_x_0 = ReactivePlannerState.create_from_initial_state(
            deepcopy(ego_state_as_initial_state),
            self.vehicle.wheelbase,
            self.vehicle.wb_rear_axle
        )

        planned_x_0 = deepcopy(self.planner_interface.x_0)

        # Compute the errors at planner states
        # TODO Log this to a file maybe
        msgs = list()
        msgs.append('\n')
        msgs.append(f"Errors between states (updated - planned) at time {updated_x_0.time_step}:")

        for attribute in [a for a in updated_x_0.used_attributes if a != "time_step"]:
            msgs.append(f"- {attribute} = {getattr(updated_x_0, attribute) - getattr(planned_x_0, attribute)}")
        msgs.append('\n')
        msgs = "\n".join(msgs)
        self.msg_logger.critical(f"{msgs}")



        # This one ONLY changes the batched-agent, not the agent synchronized in simulation
        # However, the agent will send the "updated" trajectory including the updated initial state
        self.planner_interface.record_state_list[-1] = updated_x_0
        # Setting the x_cl to None will force planner to recompute it
        # NOTE: x_cl is the state actually used by the (CPP) planner to plan the trajectories!
        self.planner_interface.planner.x_cl = None

        # Update the new cartesian current state to match the one observed in the scenario
        self.planner_interface.planner.update_externals(x_0=updated_x_0)
        # Log the updated x_cl as well. We need both to update planner_interface internal state (BAD DESIGN!)
        updated_x_cl = self.planner_interface.planner.x_cl

        # This DOES NOT automatically happen after calling update_externals
        # So we need to force it. 
        self.planner_interface.x_0 = updated_x_0
        # The X_CL is used by the CPP planner to compute the trajectories, but calling planner.update_externals was not enough 
        self.planner_interface.x_cl = updated_x_cl

        if not collision:
            if self.config_simulation.use_multiagent:
                self.scenario = hf.scenario_without_obstacle_id(scenario=deepcopy(scenario), obs_ids=[self.id])
            if global_predictions:
                self.predictions, self.visible_area = ph.filter_global_predictions(self.scenario, global_predictions,
                                                                                   self.vehicle_history[-1],
                                                                                   time_step,
                                                                                   self.config,
                                                                                   occlusion_module=self.planner_interface.occlusion_module,
                                                                                   ego_id=self.id,
                                                                                   msg_logger=self.msg_logger)
            else:
                self.predictions, self.visible_area = None, None
        else:
            self.agent_state.log_collision(time_step)
    agent_cls = getattr(simulation, "Agent")
    agent_cls._original_update_agent = agent_cls.update_agent
    agent_cls.update_agent = patched_update_agent

    def patched_log_evaluation(self, results):
        """
        Log the criticality evaluation results
         """
        data = []
        for (agent_id, t) in results.index:
            orig_pp = True if agent_id in self.original_planning_problem_id else False
            # TODO - WHY THE AGENT_ID + 10 ?!
            # data.append([self.scenario, agent_id+10, orig_pp, t] + list(results.loc[(agent_id, t)]))
            data.append([self.scenario, agent_id, orig_pp, t] + list(results.loc[(agent_id, t)]))

        text = "INSERT INTO scenario_evaluation VALUES(" + "?," *len(data[0])
        text = text[:-1] + ")"
        self.con.executemany(text, data)
        self.con.commit()

    multi_agent_log_module = getattr(simulation, "multi_agent_log")
    sim_logger_cls = getattr(multi_agent_log_module, "SimulationLogger")
    sim_logger_cls._original_log_evaluation = sim_logger_cls.log_evaluation
    sim_logger_cls.log_evaluation = patched_log_evaluation

######## Monkey Patching problematic libraries

from datetime import datetime
from cr_beamng_cosimulation.utils import LOGS_DIR
import time

# TODO Consider declaring scenario-file here as well
@click.group()
@click.pass_context
@click.option('--output-folder', type=click.Path(exists=False), required=False)
@click.option('--scenario-name', type=click.Path(exists=False), required=False)
@click.option('--verbose/--no-verbose', default=False, help='Activate verbose debugging on console')
def entry_point(ctx, output_folder: str, scenario_name: str, verbose: bool):
    if output_folder is None:
        # Defatul location
        output_folder = LOGS_DIR

    if scenario_name is None:
        dt = datetime.today()
        seconds = dt.timestamp()
        scenario_name = f"SIMULATION_{seconds}"
    
    ctx.obj = dict()
    ctx.obj['output_folder'] = output_folder
    ctx.obj['scenario_name'] = scenario_name
    ctx.obj['verbose'] = verbose
    ctx.obj['start_time'] = time.time()


    # Monkey Patch Problematic Libraries
    apply_monkey_path()


# Register the call back
# See: https://stackoverflow.com/questions/38164324/python-click-having-the-group-execute-code-after-a-command
@entry_point.result_callback()
@click.pass_context
def process_result(ctx, result, **kwargs):
    end_time = time.time()
    start_time = ctx.obj["start_time"]
    elapsed_time = end_time-start_time

    verbose = ctx.obj['verbose']
    # ctx.obj['start_time'] = time.time()
    # ctx.obj["execution_time_file"] = os.path.join(output_folder, "resimulation_time.txt")
    if verbose:
        click.echo(f'Time to execute the command {elapsed_time}')
    if "execution_time_file" in ctx.obj:
        execution_time_file = ctx.obj["execution_time_file"]
        with open(execution_time_file, "w") as output_file:
            output_file.write(f"{elapsed_time}")


# Register all the commands

# Simulate a CommonRoad scenario using CommonRoad
entry_point.add_command(simulation_commands.simulate)

# Cosimulate a CommonRoad scenario using BeamNG.tech
entry_point.add_command(simulation_commands.cosimulate_with_beamng)

# Compare low- and hi-fidelity simulations
entry_point.add_command(analysis_commands.compare)

### Visualization 
import cr_beamng_cosimulation.simulation_visualization.commands as visualization_commands

# Plot each timestep of the simulation as frame (in PNG)
entry_point.add_command(visualization_commands.plot_simulation)

# Merge all the frames in a folder using gifski
entry_point.add_command(visualization_commands.animate_simulation)


if __name__ == "__main__":
   entry_point()