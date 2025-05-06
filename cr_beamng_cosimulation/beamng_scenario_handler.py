# Contains the scritp to setup a scenario in BeamNG and other utilities
import math
import sys
import numpy as np
import time

from typing import Dict, Tuple, Any, Optional, List

from commonroad.scenario.scenario import Scenario as CR_Scenario
from commonroad.planning.planning_problem import PlanningProblem as CR_PlanningProblem, PlanningProblemSet as CR_PlanningProblemSet
from commonroad.scenario.trajectory import Trajectory as CR_Trajectory
from commonroad.scenario.state import State as CR_State, InputState as CR_InputState


from beamngpy import BeamNGpy, Scenario as BNG_Scenario, Road as BNG_Road, Vehicle as BNG_Vehicle
from beamngpy.sensors import Timer, Electrics, Damage, AdvancedIMU # TODO Add this one
from beamngpy.misc.quat import angle_to_quat

from shapely.geometry import Point

from cr_beamng_cosimulation.utils import get_color_dict_for_planning_problems, get_center_of_bbox, remap_position, map_CR_to_BNG_lane_marking
from cr_scenario_handler.planner_interfaces.frenet_interface import FrenetPlannerInterface

DEFAULT_Z_VALUE = 0.0

# Monkey patch BeamNG library. Note: beamngpy = 1.30
def apply_mokey_patch_to_beamng():
    """
    Apply the patches to BeamNG
    """

    #### Patch the file system module to return the right binaries, home folders, and the like

    import platform
    import os

    from pathlib import Path

    from beamngpy.beamng import filesystem
    from beamngpy.logging import BNGError, BNGValueError
    # Will this complicate things?
    from beamngpy.beamng.filesystem import BINARIES, logger

    filesystem.original_determine_binary = filesystem.determine_binary
    filesystem.original_determine_home = filesystem.determine_home
    filesystem.original_determine_userpath = filesystem.determine_userpath

    from pathlib import Path, PureWindowsPath

    def patched_determine_binary(home: Path) -> Path:
        # For WSL use the patched implementation.
        # Binary location must be a PosixPath since we start it from Linux, but should point to the original location of the
        #   Windows exe
        if platform.system() == "Linux":
            choice = None
            binaries = BINARIES
            for option in binaries:
                # If home is C:/BeamNG/BeamNG.tech.v0.32.4.0/Bin64/BeamNG.tech.x64.exe
                # we map it to /mnt/c/BeamNG/BeamNG.tech.v0.32.4.0/Bin64/BeamNG.tech.x64.exe
                home_as_str = str(home)
                disk = home_as_str.split(":")[0]
                path = home_as_str.split(":")[1]
                home = Path(f"/mnt/{disk.lower()}" + '/'.join(path.split("\\")))

                binary = home / option

                if binary.exists():
                    choice = binary
                    break

            if not choice:
                raise BNGError(
                    "No BeamNG binary found in BeamNG home. Make "
                    "sure any of these exist in the BeamNG home "
                    f'folder: {", ".join(binaries)}'
                )

            logger.debug(f"Determined BeamNG.* binary to be: {choice}")
            return choice
        else:
            # For Windows use the original implementation
            return filesystem.original_determine_binary(home)

    def patched_determine_home(home: str | None) -> Path:
        # WSL Patch.

        if platform.system() == "Linux":
            if not home:
                home = os.getenv("BNG_HOME")
            if not home:
                raise BNGValueError(
                    "No BeamNG home folder given. Either specify "
                    "one in the constructor of `BeamNGpy` or define an "
                    'environment variable "BNG_HOME" that '
                    "points to where your copy of BeamNG.* is."
                )

            return Path(home)
        else:
            # For Windows invoke the original code
            return filesystem.original_determine_home(home)

    def patched_determine_userpath(binary: Path) -> Path:
        assert not platform.system() == "Linux", "Cannot invoke this on WSL!"
        return filesystem.filesystem.original_determine_userpath(binary)

    filesystem.determine_binary = patched_determine_binary
    filesystem.determine_home = patched_determine_home
    # We do not patch this one because we assume user folder is passed as input no matter what
    filesystem.determine_userpath = patched_determine_userpath

    # Patch the start method of the BeamNG class to first starting the simulator and then connecting to it

    import beamngpy
    from beamngpy.connection import Connection
    from typing import List
    from time import sleep
    import subprocess

    def patched_open(
        self,
        extensions: List[str] | None = None,
        *args: str,
        launch: bool = True,
        debug: bool | None = None,
        listen_ip: str = "127.0.0.1",
        **opts: str,
    ):

        if launch:

            self.logger.info("Opening BeamNGpy instance.")
            arg_list = list(args)

            if debug is None:
                debug = self.debug
            if debug == True:
                arg_list.append("-tcom-debug")
            elif debug == False:
                arg_list.append("-no-tcom-debug")

            # Force showing the console
            arg_list.extend(["-console"])
            arg_list.extend(("-tcom-listen-ip", listen_ip))

            self._start_beamng(extensions, *arg_list, **opts)
            sleep(10)

        # Make sure that you give the windows host in the configuration file. Run on the WSL console the following command: ip route | grep default | awk '{print $3}'
        self.connection = Connection(self.host, self.port)

        # try to connect to existing instance
        # print(f"Connecting to BeamNG at {self.host}:{self.port}")
        connected = self.connection.connect_to_beamng(tries=1, log_tries=False)
        # print(f"Connected {self.host}:{self.port}")

        assert connected, "Cannot connect to BeamNGpy"

        self.logger.info("BeamNGpy successfully connected to existing BeamNG instance.")

        self._load_system_info()

        return self

    def patched__start_beamng(
        self, extensions: List[str] | None, *args: str, **opts: str
    ) -> None:
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        home = filesystem.determine_home(self.home)
        if self.binary:
            binary = home / self.binary
            if not binary.is_file():
                raise BNGError(
                    f"The BeamNG binary {binary} was not found in BeamNG home."
                )
        else:
            binary = filesystem.determine_binary(home)

        if sys.platform == "linux" and self.user:
            # WSL Patch - This does not fix the permission issue when BeamNG needs to create another logfile ...
            from pathlib import PureWindowsPath
            window_userpath = PureWindowsPath(self.user)
            userpath = window_userpath
        else:
            userpath = (
                Path(self.user) if self.user else filesystem.determine_userpath(binary)
            )


        call = self._prepare_call(str(binary), userpath, extensions, *args, **opts)

        # Start as a regular windows exe but disable the annoying console messages
        # We need to retry starting because of the log creation issue of BeamNG
        MAX_RETRY = 5
        for retry in range(MAX_RETRY):
            self.process = subprocess.Popen(call, stdout=subprocess.DEVNULL, stdin=subprocess.PIPE)
            sleep(5)
            myProcessIsRunning = self.process.poll() is None
            if not myProcessIsRunning:
                # print(f"BeamNG did not started!")
                self.process.kill()
            else:
                break

        self.logger.info("Started BeamNG.")

    beamngpy.BeamNGpy.__original_open = beamngpy.BeamNGpy.open
    beamngpy.BeamNGpy.__original__start_beamng = beamngpy.BeamNGpy._start_beamng

    beamngpy.BeamNGpy.open = patched_open
    beamngpy.BeamNGpy._start_beamng = patched__start_beamng

# TODO Consider moving this into another module
apply_mokey_patch_to_beamng()

class BeamNGScenario():
    """
    DTO for storing meta-data/references to a BeamNG scenario.
    """
    def __init__(self, scenario: BNG_Scenario, vehicles_dict: Dict[int, BNG_Vehicle], imus_dict: Dict[int, AdvancedIMU]):
        self.scenario = scenario
        self.vehicles_dict = vehicles_dict
        self.imus_dict = imus_dict

class BeamNGScenarioHandler():
    """
    This class is a wrapper around BeamNG Python API.
    """

    def __init__(self, beamng_host: str, beamng_port: int, beamng_home_folder: str, beamng_user_folder: str,
                 visualize_debug_spheres: bool = False):
        self.host = beamng_host
        self.port = beamng_port
        self.beamng_home_folder = beamng_home_folder
        self.beamng_user_folder = beamng_user_folder

        self._cr_step_duration = 0.1 # sec
        self._steps_per_second = 60  # One step corresponds to 0.1 sec
        self._beamng_to_cr_steps_mapping = 6

        self.beamng = BeamNGpy(self.host, self.port, home=self.beamng_home_folder, user=self.beamng_user_folder,
                               debug=True)
        self.running = False

        # Visualize the debug spheres
        self.visualize_debug_spheres = visualize_debug_spheres
        
        # Dict[planner_id, Dict[color_str, spheres]]
        self.cr_trajectories = dict()
        self.cr_ref_paths = dict()

    def start_simulator_if_not_running(self):
        if not self.running:
            # Note this trick for WSL
            if sys.platform == "linux":
                # Note: We need to listen connection from any IP
                self.beamng.open(listen_ip='0.0.0.0', launch=True)
            else:
                self.beamng.open(launch=True)

        self.running = True

    def stop_simulator(self):
        try:
            self.beamng.close()
        finally:
            self.running = False

    def draw_ref_path(self, planner_id: int, ref_path):
        if not self.visualize_debug_spheres:
            return
        
        assert self.running, "The simulator is not running"

        sphere_coordinates = []
        sphere_radii = []
        sphere_colors = []

        for position in ref_path:
            sphere_coordinates.append((position[0], position[1], 1.0))
            sphere_radii.append(0.10)
            sphere_colors.append("green")

        return self.beamng.debug.add_spheres(sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=2.0)
    
    def refresh_trajectory_in_simulation(self, planner_id: int, cr_trajectory: CR_Trajectory, color="blue", size_factor=1.0):
        
        if not self.visualize_debug_spheres:
            return
        
        assert self.running, "The simulator is not running"
        
        # Use the color to plot many different trajectories for the same agent

        # Hide them if visible
        if planner_id in self.cr_trajectories and color in self.cr_trajectories[planner_id] and self.cr_trajectories[planner_id][color]:
            self.beamng.debug.remove_spheres(self.cr_trajectories[planner_id][color])
            self.cr_trajectories[planner_id][color] = None

        # Ensure entries in the dictionaries are there
        if planner_id not in self.cr_trajectories:
            self.cr_trajectories[planner_id] = dict()

        if color not in self.cr_trajectories[planner_id]:
            self.cr_trajectories[planner_id][color] = None

        # Visualize them, if anything with the rigth color
        self.cr_trajectories[planner_id][color] = self._visualize_trajectory_in_simulation(cr_trajectory, color, size_factor)

    def refresh_front_position(self, planner_id: int, front_position):
        if not self.visualize_debug_spheres:
            return
        
        assert self.running, "The simulator is not running"

        self.refresh_trajectory_in_simulation(planner_id, [front_position], color="orange", size_factor=2.0)

    def _visualize_trajectory_in_simulation(self, cr_trajectory: CR_Trajectory|List, color: str, size_factor: float):
        """
        Visualize the planned trajectory using debug spheres
        :param cr_trajectory:
        :return:
        """
        assert self.running, "The simulator is not running"

        sphere_coordinates = []
        sphere_radii = []
        sphere_colors = []

        
        if type(cr_trajectory) != list:
            state_list = cr_trajectory.state_list
        else:
            state_list = cr_trajectory

        for state in state_list:
            sphere_coordinates.append((state.position[0], state.position[1], 1.0))
            sphere_radii.append(0.10 * size_factor)
            sphere_colors.append(color)

        # Make the spheres float 2m above the ground and return them
        return self.beamng.debug.add_spheres(sphere_coordinates, sphere_radii, sphere_colors, cling=True, offset=2.0)

    def step(self, steps: int=1, sleep=False):
        assert self.running, "BeamNG must be running."
        self.beamng.step(steps * self._beamng_to_cr_steps_mapping)
        # Simulate a blocking call assuming real time == simulated time
        if sleep:
            time.sleep(steps * (1.1 * self._cr_step_duration))

    # TODO Split road generation and vehicle placement
    def setup_and_start_scenario_in_beamng(self,
                                           cr_scenario: CR_Scenario, cr_planning_problem_set: CR_PlanningProblemSet,
                                           config_vehicle,
                                           config_beamng) -> BeamNGScenario:
        """
        Render the CommonRoad scenario in BeamNG and place the vehicles in the expected position.
        Currently, vehicles' initial state MUST be "standing still" (v=0, a=0, yaw_rate=0).
        """

        assert self.running, "Simulator is not running"

        try:
            # This will fail if no scenarios are loaded
            self.beamng.get_current_scenario().close()
        except:
            pass

        # Name the new scenario based on the information of the CommonRoad scenario
        beamng_scenario_name = f"CR_{cr_scenario.scenario_id}"
        beamng_scenario = BNG_Scenario(config_beamng.beamng_level_name, beamng_scenario_name)

        # Generate the map according to the CR Scenario Lanelet Network by mapping each lanelet to a road (segment) in BeamNG
        # The width of the node is computed using left/right nodes of each lanelet
        for lanelet in cr_scenario.lanelet_network.lanelets:
            road_nodes = []
            for left, center, right in zip(lanelet.left_vertices, lanelet.center_vertices, lanelet.right_vertices):
                lane_width = Point(left).distance(Point(right))
                road_nodes.append((center[0], center[1], DEFAULT_Z_VALUE, lane_width))

            # Render at the bottom (priority 10)
            # TODO We assume those materials exist.
            beamng_road = BNG_Road('road_asphalt_light', interpolate=False,
                                     rid=f"lane_{lanelet.lanelet_id}",
                                     over_objects=True,
                                     drivability=-1, render_priority=10)

            # Create also the invisible lane on top of it, this is the actual road the NAVI sees
            # Render at the top (priority 5)
            invisible_beamng_road = BNG_Road('road_invisible', interpolate=False,
                                               rid=f"invisible_lane_{lanelet.lanelet_id}",
                                               over_objects=True,
                                               drivability=1, render_priority=1)

            beamng_road.add_nodes(*road_nodes)
            invisible_beamng_road.add_nodes(*road_nodes)

            beamng_scenario.add_road(beamng_road)
            beamng_scenario.add_road(invisible_beamng_road)
        
        # Generate additional details, such as lanemarking
        # The width of the lanemarking
        for lanelet in cr_scenario.lanelet_network.lanelets:
            # Those info are NOT encoded inside the scenario description but usually provided while rendering
            is_left_yellow = False
            is_right_yellow = False
            
            # Note that if no linemarking is specified, we should not render anything in BNG otherwise is might override other lanemarkings
            material_lane_marking_left, left_width = map_CR_to_BNG_lane_marking(lanelet.line_marking_left_vertices, is_left_yellow)
            material_lane_marking_right, right_width = map_CR_to_BNG_lane_marking(lanelet.line_marking_right_vertices, is_right_yellow)

            left_lanemarking_road_nodes = []
            right_lanemarking_road_nodes = []
            for left, right in zip(lanelet.left_vertices, lanelet.right_vertices):
                # TODO Width is default 0.1 m
                lane_width = 0.1
                left_lanemarking_road_nodes.append((left[0], left[1], DEFAULT_Z_VALUE, left_width))
                right_lanemarking_road_nodes.append((right[0], right[1], DEFAULT_Z_VALUE, right_width))

            # Render at the top (priority 1)
            if material_lane_marking_left is not None:
                left_beamng_road = BNG_Road(material_lane_marking_left, interpolate=False,
                                        rid=f"lane_{lanelet.lanelet_id}_line_marking_left",
                                        over_objects=True,
                                        drivability=-1, render_priority=1)
                left_beamng_road.add_nodes(*left_lanemarking_road_nodes)
                beamng_scenario.add_road(left_beamng_road)

            if material_lane_marking_right is not None:
                right_beamng_road = BNG_Road(material_lane_marking_right, interpolate=False,
                                        rid=f"lane_{lanelet.lanelet_id}_line_marking_right",
                                        over_objects=True,
                                        drivability=-1, render_priority=1)

                right_beamng_road.add_nodes(*right_lanemarking_road_nodes)
                beamng_scenario.add_road(right_beamng_road)

        # Instantiate the vehicles
        # Colors and Licence Plates reflect their color and planning problem id in CR
        vehicles_dict = {}
        imus_dict = {}

        for planning_problem_id, color in get_color_dict_for_planning_problems(cr_planning_problem_set).items():

            # Get a reference to the planning problem object
            planning_problem: CR_PlanningProblem = cr_planning_problem_set.planning_problem_dict[planning_problem_id]

            # This must be a string and cannot start with a digit
            vehicle_id = f"vehicle_{planning_problem_id}"

            # TODO Extend such that every one has its own model instead
            beamng_vehicle_model = config_vehicle.beamng_vehicle_model

            beamng_vehicle = BNG_Vehicle(vehicle_id, model=beamng_vehicle_model, color=color,
                                           licenseText=f"CR {planning_problem.planning_problem_id}")
            # Attach the sensors
            # Damage sensor
            beamng_vehicle.sensors.attach('damage', Damage())
            # Speeometer and the like - get steering
            beamng_vehicle.sensors.attach('electrics', Electrics())
            # Access to the simulation time
            beamng_vehicle.sensors.attach('timer', Timer())

            # Set the vehicle initial state according to CR scenario
            state = planning_problem.initial_state
            x = state.position[0]
            y = state.position[1]
            z = DEFAULT_Z_VALUE
            orientation_rad = state.orientation
            orientation_deg = math.degrees(orientation_rad)

            # Adjust the position according to the vehicle model such that the center of the vehicle corresponds to
            # the initial state from CR Scenario
            x, y, z = remap_position(beamng_vehicle_model, (x, y, z), angle_deg=orientation_deg)

            # Adjust the vehicle rotation paying attention that CR and BeamNG use different Cartesian systems
            angle_deg = - orientation_deg - 90
            # BeamNG uses Quaternion
            rot_quat = angle_to_quat((0, 0, angle_deg))

            # Add the vehicle to the scenario in the expected position, rotation, cling=True adjusts z to match ground
            beamng_scenario.add_vehicle(beamng_vehicle, pos=(x, y, z), rot_quat=rot_quat, cling=True)
            # Note we use planning_problem_id to enable looking vehicles up from CR data
            vehicles_dict[planning_problem_id] = beamng_vehicle

        # Render the scenario to disk
        beamng_scenario.make(self.beamng)

        # Load the scenario inside the simulator
        self.beamng.scenario.load(beamng_scenario)

        # Configure the simulator for synchronous simulation
        self.beamng.settings.set_deterministic(self._steps_per_second)

        # Make sure the simulation start with simulator paused (timer sensors is 0.0)
        self.beamng.control.pause()

        # Start the scenario
        self.beamng.scenario.start()

        # TODO Check if all the sensors should be attached at this point...

        ## Yaw Rate and Accelerations - This sensor can be attached ONLY after starting the simulation?!
        for vehicle_id, beamng_vehicle in vehicles_dict.items():
            # TODO: Why this sensor works differently!!! :(
            imus_dict[vehicle_id] = AdvancedIMU("aimu", self.beamng, beamng_vehicle, gfx_update_time=0.01)

        # Place the general camera on top of the vehicle (TODO Later we need to decide what to do!)
        # Set the user camera relative to the focused vehicle (the only one in the scenario)
        # Set the scenario camera to be birdview over the vehicle
        self.beamng.camera.set_relative(pos=(0, 0, 30), dir=(0, 0, -1))

        # Collect the distance between the position of the vehicle and its front
        bng_scenario = BeamNGScenario(beamng_scenario, vehicles_dict, imus_dict)
#
        # TODO Improve this with a Class or just return self.beamng since all the other variables can be obtained calling its functions
        #
        return bng_scenario

    # TODO Why there are two of such methods setup_and vs generate_and ?!!

    # TODO Split road generation and vehicle placement
    # TODO This is here only for backwards compatibility. Deprecated!
    # TODO THIS METHOD IS DEPRECATED
    def generate_and_start_cr_scenario_in_beamng(self,
                                                 cr_scenario: CR_Scenario, cr_planning_problem_set: CR_PlanningProblemSet,
                                                 commonroad_planners: Dict[int, FrenetPlannerInterface],
                                                 beamng_level_name: str = "tech_ground", beamng_scenario_name: str = None):
        """
        Given the cr_scenario and cr_planning_problem_set render the scenario in BeamNG placing the vehicles in the expected position.
        TODO: currently, the vehicles' initial state is "standing still" (v=0, a=0, yaw_rate=0). We'll address the issue to warm up vehicles in the future (see https://github.com/alessiogambi/cr-beamng-cosimulation/issues/13).
        TODO: planning problems are not really necessary at this points, since the cr_scenario contains a Dynamic Obstacle corresponding to each one of them.
        TODO: Toggle camera to bird and/or focus on vehicles
        """

        assert self.running, "Simulator is not running"

        try:
            # This will fail if no scenarios are loaded
            self.beamng.get_current_scenario().close()
        except:
            pass

        # Create the new scenario
        beamng_scenario_name = beamng_scenario_name if beamng_scenario_name else f"CR_{cr_scenario.scenario_id}"
        beamng_scenario = BNG_Scenario(beamng_level_name, beamng_scenario_name)

        # Generate the map according to the CR Scenario Lanelet Network by mapping each lanelet to a road (segment) in BeamNG
        # The width of the node is computed using left/right nodes of each lanelet
        for lanelet in cr_scenario.lanelet_network.lanelets:
            road_nodes = []
            for left, center, right in zip(lanelet.left_vertices, lanelet.center_vertices, lanelet.right_vertices):
                lane_width = Point(left).distance(Point(right))
                road_nodes.append((center[0], center[1], DEFAULT_Z_VALUE, lane_width))

            # Render at the bottom (priority 10)
            # TODO We assume those materials exist.
            beamng_road = BNG_Road('road_asphalt_light', interpolate=False,
                                     rid=f"lane_{lanelet.lanelet_id}",
                                     over_objects=True,
                                     drivability=-1, render_priority=10)

            # Create also the invisible lane on top of it, this is the actual road the NAVI sees
            # Render at the top (priority 1)
            invisible_beamng_road = BNG_Road('road_invisible', interpolate=False,
                                               rid=f"invisible_lane_{lanelet.lanelet_id}",
                                               over_objects=True,
                                               drivability=1, render_priority=1)

            beamng_road.add_nodes(*road_nodes)
            invisible_beamng_road.add_nodes(*road_nodes)

            beamng_scenario.add_road(beamng_road)
            beamng_scenario.add_road(invisible_beamng_road)

        # Instantiate the vehicles
        # Colors and Licence Plates reflect their color and planning problem id in CR
        vehicles_dict = {}
        imus_dict = {}

        for planning_problem_id, color in get_color_dict_for_planning_problems(cr_planning_problem_set).items():

            # Get a reference to the driving agent
            commonroad_planner: FrenetPlannerInterface = commonroad_planners[planning_problem_id]

            # Get a reference to the planning problem object
            planning_problem: CR_PlanningProblem = cr_planning_problem_set.planning_problem_dict[planning_problem_id]

            # This must be a string and cannot start with a digit
            vehicle_id = f"vehicle_{planning_problem_id}"

            # Default vehicle model.
            beamng_vehicle_model = commonroad_planner.config_sim.vehicle.beamng_vehicle_model # 'etk800'

            beamng_vehicle = BNG_Vehicle(vehicle_id, model=beamng_vehicle_model, color=color,
                                           licenseText=f"CR {planning_problem.planning_problem_id}")
            
            # Attach the sensors
            # Damage sensor
            beamng_vehicle.sensors.attach('damage', Damage())
            # Speeometer and the like - get steering
            beamng_vehicle.sensors.attach('electrics', Electrics())
            # Access to the simulation time
            beamng_vehicle.sensors.attach('timer', Timer())

            # Set the vehicle initial state according to CR scenario
            state = planning_problem.initial_state
            x = state.position[0]
            y = state.position[1]
            z = DEFAULT_Z_VALUE
            orientation_rad = state.orientation
            orientation_deg = math.degrees(orientation_rad)

            # Adjust the position according to the vehicle model such that the center of the vehicle corresponds to
            # the initial state from CR Scenario
            x, y, z = remap_position(beamng_vehicle_model, (x, y, z), angle_deg=orientation_deg)

            # Adjust the vehicle rotation paying attention that CR and BeamNG use different Cartesian systems
            angle_deg = - orientation_deg - 90
            # BeamNG uses Quaternion
            rot_quat = angle_to_quat((0, 0, angle_deg))

            # Add the vehicle to the scenario in the expected position, rotation, cling=True adjusts z to match ground
            beamng_scenario.add_vehicle(beamng_vehicle, pos=(x, y, z), rot_quat=rot_quat, cling=True)
            # Note we use planning_problem_id to enable looking vehicles up from CR data
            vehicles_dict[planning_problem_id] = beamng_vehicle

        # Render the scenario to disk
        beamng_scenario.make(self.beamng)

        # Load the scenario inside the simulator
        self.beamng.scenario.load(beamng_scenario)

        # Configure the simulator for synchronous simulation
        self.beamng.settings.set_deterministic(self._steps_per_second)

        # Make sure the simulation start with simulator paused (timer sensors is 0.0)
        self.beamng.control.pause()

        # Start the scenario
        self.beamng.scenario.start()

        # TODO Check if all the sensors should be attached at this point...

        ## Yaw Rate and Accelerations - This sensor can be attached ONLY after starting the simulation?!
        for vehicle_id, beamng_vehicle in vehicles_dict.items():
            # TODO: Why this sensor works differently!!! :(
            imus_dict[vehicle_id] = AdvancedIMU("aimu", self.beamng, beamng_vehicle, gfx_update_time=0.01)
            
            # Set the vehicle mode - need to be in a running simulation
            beamng_vehicle.ai_set_mode("manual")

        # Place the general camera on top of the vehicle (TODO Later we need to decide what to do!)
        # Set the user camera relative to the focused vehicle (the only one in the scenario)
        # Set the scenario camera to be birdview over the vehicle
        self.beamng.camera.set_relative(pos=(0, 0, 30), dir=(0, 0, -1))

        #
        # TODO Improve this with a Class or just return self.beamng since all the other variables can be obtained calling its functions
        #
        return {
            "beamng": self.beamng,
            "beamng_scenario": beamng_scenario,
            "vehicles_dict": vehicles_dict,
            "imus_dict": imus_dict
        }

    # TODO Remove time_step and compute the actual time_Step using scenario.dt and logical time from Timer sensor
    def get_state_of_beamng_vehicles(self, time_step: int, bng_scenario: BeamNGScenario, use_extended_attributes=False) -> Tuple[Dict[int, CR_State], Dict[int, bool]]:
        """
        Poll each vehicle sensors (state, electricity, damage) and build a CommonRoad state with the data in it
        """
        vehicles_dict: Dict[int, BNG_Vehicle] = bng_scenario.vehicles_dict
        imus_dict: Dict[int, AdvancedIMU] = bng_scenario.imus_dict

        # Double checking the current scenario might not be necessary and causes issues and slowed down executions
        # This one seems to open a new connection
        # beamng_scenario = self.beamng.scenario.get_current()
        # Update the vehicles state to match the one inside BeamNG simulation
        # beamng_scenario.update()

        # Create a state object for each vehicle
        states = {}
        # Report any damage value in case of collision. 
        # TODO: This might be unreliable for low-speed and swipe collisions
        damages = {}

        for vehicle_id, beamng_vehicle in vehicles_dict.items():
            # Pull the data from the vehicle
            beamng_vehicle: BNG_Vehicle
            beamng_vehicle.poll_sensors()

            # CommonRoad state
            # Attributes of first state: ['time_step', 'position', 'orientation', 'velocity', 'acceleration', 'yaw_rate', 'slip_angle'].
            commonroad_state = CR_State()

            # Force the logical time
            commonroad_state.time_step = time_step

            # Add position measured as the center of the vehicle bounding box
            center = get_center_of_bbox(beamng_vehicle.get_bbox())

            commonroad_state.position = np.array((center.x, center.y))
            # Add rotation
            quat = beamng_vehicle.sensors['state']['rotation']
            t0 = 2.0 * (quat[3] * quat[2] + quat[0] * quat[1])
            t1 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
            yaw = math.atan2(t1, t0) + math.pi
            # Normalize to [-pi, pi]
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            commonroad_state.orientation = yaw

            # Add velocity
            (v_x, v_y, v_z) = beamng_vehicle.sensors['state']['vel']
            commonroad_state.velocity = math.sqrt(v_x ** 2 + v_y ** 2 + v_z ** 2)
            # Todo: Add acceleration

            # We cannot add all the attributes here otherwise CommonRoad complains that the trajectory states do not have the same attributes!
            electrics = beamng_vehicle.sensors['electrics']

            # NOTE: This requires to patch wheels.lua and electrics.lua. wheels.lua must set the value using the "local function getWheelAngles()"
            assert "wheel_angle" in electrics, "wheel angle is not defined. Please check whether you have correctly patches BeamNG.tech"
                
            commonroad_state.steering_angle = electrics["wheel_angle"]
            
            is_damaged = True if beamng_vehicle.sensors['damage']['damage'] > 0.0 else False

            # dt = previous_state.simulation_time
            # commonroad_state.yaw_rate = yaw - previous_state.yaw  / dt if previous_state is not None else 0.0

            # TODO IMUS for acceleration and YawRate
            # TODO This may not be accurate. Probably we should use the Advanced IMU
            # accX, accY = electrics["accXSmooth"], electrics["accYSmooth"]
            # acceleration = (accX ** 2 + accY ** 2) ** 0.5
            # commonroad_state.acceleration = acceleration
            aimu  = imus_dict[vehicle_id]
            imu_data = aimu.poll()  # Fetch the latest readings from the sensor.
            sorted_keys = sorted(imu_data)
            sum = 0
            for key in sorted_keys[-5:]:
                sum += imu_data[key]['angVel'][2]
            yaw_rate = sum / 5
            # imu_data[x]['angVel'][2] is angular velocity, positive is a left turn, negative a right turn
            commonroad_state.yaw_rate = yaw_rate
            sum = 0
            for key in sorted_keys[-5:]:
                sum += imu_data[key]['accSmooth'][0]
            accel = sum / 5
            commonroad_state.acceleration = accel
            
            states[vehicle_id] = commonroad_state
            damages[vehicle_id] = is_damaged

            # Extended attributes break CR utilities so we use only when necessary
            if use_extended_attributes:
                simulation_time = beamng_vehicle.sensors['timer']['time']
                commonroad_state.simulation_time = simulation_time
                commonroad_state.front_position = beamng_vehicle.sensors['state']['front']

        return states, damages
    
    def remove_vehicle(self, vehicle_id: int, bng_scenario: BeamNGScenario):
        vehicles_dict: Dict[int, BNG_Vehicle] = bng_scenario.vehicles_dict
        if vehicle_id in vehicles_dict:
            self.beamng.despawn_vehicle(vehicles_dict[vehicle_id])
            del vehicles_dict[vehicle_id]