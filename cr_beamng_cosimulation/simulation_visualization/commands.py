from typing import Optional, List, Dict, Tuple

import subprocess
import click
import os
import glob
import csv
import enlighten
import inspect
import sys

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.colors as mc

import colorsys


from commonroad.scenario.scenario import Scenario, DynamicObstacle
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.common.file_reader import XMLFileReader
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction

from commonroad.visualization.mp_renderer import MPDrawParams, MPRenderer


# TODO Move to common.utils
# CODE CLONE!
def _get_cmap(n, name='hsv'):
    """Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name."""
    return plt.cm.get_cmap(name, n)

def alter_color(color, amount=0.5):
    """
    Lightens or darkens the given color by multiplying (1-luminosity) by the given amount.
    Input can be matplotlib color string, hex string, or RGB tuple.

    See: https://stackoverflow.com/questions/37765197/darken-or-lighten-a-color-in-matplotlib
    """
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])

def visualize_the_scenario(scenario: Scenario, planning_problem_set: PlanningProblemSet, output_folder: str,
                           time_step: int, last_time_step: int,
                           size_inch: int, format: str,
                           x_lim: Tuple[float, float], y_lim: Tuple[float, float],
                           label: Optional[str] = None):
    """
    Plot the scenario at the given time_step
    """

    obstacles = sorted(scenario.obstacles, key= lambda o : o.obstacle_id)

    unique_colors = _get_cmap(len(obstacles), 'rainbow')

    color_mapping = dict()
    for i, obstacle in enumerate(obstacles):
        color_mapping[obstacle.obstacle_id] = colors.to_hex(unique_colors(i))
    
    figsize = (size_inch, size_inch)
    fig = plt.figure(figsize=figsize)
    ax = fig.gca()
    rnd = MPRenderer(ax=ax)

    # set renderer draw params - This is important if there are traffic lights in the scenario
    rnd.draw_params.time_begin = time_step
    rnd.draw_params.time_end = last_time_step

    # Draw the road but not the obstacles
    # TODO We do not have reference path, do we? -> Actually we have them in the tranches!
    scenario.lanelet_network.draw(rnd)

    # Draw Occupancy of the vehicles using the appropriate colors.
    for obstacle in obstacles:

        if obstacle.occupancy_at_time(time_step) is not None:
            # Set obstacle parameters
            obs_params = MPDrawParams()
            obs_params.dynamic_obstacle.draw_icon = True
            obs_params.dynamic_obstacle.show_label = True
            
            # We need both otherwise begin and end otherwise it will not show it
            obs_params.dynamic_obstacle.time_begin = time_step
            obs_params.dynamic_obstacle.time_end = last_time_step

            obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = color_mapping[obstacle.obstacle_id]
            obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "black"
            # Ensure this is draw above everything else
            obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.zorder = 100
            obs_params.dynamic_obstacle.vehicle_shape.occupancy.shape.opacity = 1

            obs_params.dynamic_obstacle.draw_direction = True
            obs_params.dynamic_obstacle.draw_signals = True
            
            # Draw a line instead of a set of points using the same color of the vehicle
            # Highlight better the planned trajectories (bold lines)
            obs_params.dynamic_obstacle.trajectory.draw_trajectory = False
            obs_params.dynamic_obstacle.trajectory.draw_continuous = True
            obs_params.dynamic_obstacle.trajectory.facecolor = color_mapping[obstacle.obstacle_id] # "black"
            obs_params.dynamic_obstacle.trajectory.line_width = 1.0
            # obs_params.dynamic_obstacle.trajectory.zorder = 80
            # Static obstacles have always the same color
            # obs_params.static_obstacle.show_label = True
            # obs_params.static_obstacle.occupancy.shape.facecolor = "#A30000"
            # obs_params.static_obstacle.occupancy.shape.edgecolor = "#756F61"

            # Finally render it
            obstacle.draw(rnd, draw_params=obs_params)

            # Show goal areas (until the vehicle is there)
            # Set Goal area parameters
            planning_problem = planning_problem_set.planning_problem_dict[obstacle.obstacle_id]
            goal_region = planning_problem.goal

            goal_region_params = MPDrawParams()
            goal_region_params.goal_region.shape.facecolor = color_mapping[obstacle.obstacle_id]
            goal_region_params.goal_region.shape.opacity = 0.5

            goal_region.draw(rnd, draw_params=goal_region_params)

    # Render the scenario onto the plot
    rnd.render()
    
    # Show time/timesteps
    middle_x = 0.5
    top_y = 0.9
    # ax.text(middle_x, top_y, f"Time: {str(round(float(time_step * scenario.dt), 1)).rjust(5,' ')} sec", horizontalalignment='center', verticalalignment='center', transform=ax.transAxes)

    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)

    # Add title
    if label is not None:
        plt.title(f"{label}")

    # TODO Make the format configurable if needed
    plot_file = os.path.join(output_folder, f"frame_{str(time_step).rjust(5, '0')}.{format}")
    plt.savefig(f"{plot_file}", bbox_inches='tight')
    plt.close()

import numpy as np

# focus the plotting on a specific vehicle (or replot for a specific vehicle)
@click.command()
@click.pass_context
@click.option("--original-scenario", "original_scenario_file", required=True, type=click.Path(exists=True, file_okay=True), help="The original scenario")
@click.option("--executed-scenario", "executed_scenario_file", required=True, type=click.Path(exists=True, file_okay=True), help="The simulated scenario")
@click.option("--label", required=False, type=str, help="A label to use as a title in the plots")
@click.option("--start-from", required=False, type=int, help="The time_step at which plotting starts")
@click.option("--end-at", required=False, type=int, help="The time_step at which plotting ends")
@click.option('--size', "size_inch", required=False, type=int, default=10, help='Size of the (squared) figure in inches')
@click.option('--format', "format", required=False, type=click.Choice(["png", "svg", "pdf", "jpg", "jpeg"]), default="png", help='format of the plot')
def plot_simulation(ctx, original_scenario_file: str, executed_scenario_file: str, label:str, start_from: Optional[int], end_at: Optional[int], size_inch: int, format: str): #, selected_vehicles: Optional[List[int]]):
    """
    Plot all the frames of the executed scenario
    """
    # Extract the options from ctx
    output_folder = ctx.obj["output_folder"]
    os.makedirs(output_folder, exist_ok=True)

    verbose = ctx.obj["verbose"]
    
    # Executed scenario should only have obstacles that correspond to the monitored behavior of vehicles during the simulation
    original_scenario, planning_problem_set = XMLFileReader(original_scenario_file).open(lanelet_assignment=True)
    executed_scenario, _ = XMLFileReader(executed_scenario_file).open(lanelet_assignment=True)

    first_time_step = min([o.initial_state.time_step for o in executed_scenario.obstacles])
    last_time_step = max([ max([s.time_step for s in o.prediction.trajectory.state_list]) for o in executed_scenario.obstacles])

    # Set intervals
    initial_time_step = first_time_step if start_from is None else start_from
    # We cannot start before the first timestep?
    assert first_time_step <= initial_time_step, "We cannot plot starting before the scenario starts"

    final_time_step = last_time_step if end_at is None else end_at
    assert last_time_step >= final_time_step, "We cannot plot after the scenario ends"

    manager = enlighten.Manager()
    frame_pbar = manager.counter(desc='Generated Frames', total=(final_time_step-initial_time_step))

    if verbose:
        click.echo(f"Plotting scenario {executed_scenario_file} between {initial_time_step} and {final_time_step}")
        click.echo(f"Plots will be stored at {output_folder}")

    # Compute the limits once for all

    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')
    
    for lanelet in original_scenario.lanelet_network.lanelets:
        for vertices in [lanelet.left_vertices, lanelet.center_vertices, lanelet.right_vertices]:
            min_x_val_vertices, max_x_val_vertices = np.min(vertices[:,0]), np.max(vertices[:,0])
            min_y_val_vertices, max_y_val_vertices = np.min(vertices[:,1]), np.max(vertices[:,1])

            min_x = min([min_x, min_x_val_vertices])
            max_x = max([max_x, max_x_val_vertices])
            min_y = min([min_y, min_y_val_vertices])
            max_y = max([max_y, max_y_val_vertices])

    margin = 20

    x_lim = (min_x - margin, max_x + margin)
    y_lim = (min_y - margin, max_y + margin)

    # For each agent, compute the metrics
    for time_step in frame_pbar(range(initial_time_step, final_time_step+1)):
        visualize_the_scenario(executed_scenario, planning_problem_set, output_folder, time_step, last_time_step, size_inch, format, x_lim, y_lim,
                               label=label)


def _is_tool(name):
    """Check whether name is on PATH and marked as executable."""
    from shutil import which
    return which(name) is not None

@click.command()
@click.pass_context
@click.option("--plots-folder", required=True, type=click.Path(exists=True, dir_okay=True), help="The folder containing the plots to use")
def animate_simulation(ctx, plots_folder: str):
    """
    Combine the frames into an high-quality gif
    """
    # Extract the options from ctx
    output_folder = ctx.obj["output_folder"]
    os.makedirs(output_folder, exist_ok=True)
    
    verbose = ctx.obj["verbose"]

    assert _is_tool("gifski"), "Gifski is not available on the PATH. Install and add it to the path"
    
    # Single process
    # gifski -o simulation_1741866740.gif simulation_1741866740/tranches/00000/logs/plots/*.png
    default_name = "simulation.gif"
    output_file = os.path.join(output_folder, default_name)

    exit_code = subprocess.check_call(f"gifski -o {output_file} {plots_folder}/*.png", shell=True)
    
    assert exit_code == 0, "Something went wrong while generating the gif"


