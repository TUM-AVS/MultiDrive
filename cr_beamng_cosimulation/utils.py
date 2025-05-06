import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as colors

import math
import os

import numpy as np

from omegaconf.omegaconf import OmegaConf, DictConfig

from beamngpy.types import Color, Float2, Float3
from typing import Dict, Tuple

from commonroad.scenario.lanelet import LineMarking as CR_LineMarking
from commonroad.planning.planning_problem import PlanningProblem as CR_PlanningProblem, PlanningProblemSet as CR_PlanningProblemSet
from commonroad.scenario.state import State as CR_State

from beamngpy import BeamNGpy, Vehicle as Beamng_Vehicle
from beamngpy.sensors import AdvancedIMU
from shapely.geometry import LineString

# Frenetix-Motion-Planner location
# Note: since commands is one level deeper we need to look for ../../
FRENETIX_MODULE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Frenetix-Motion-Planner")
# Store here all the results
LOGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "executed-simulations")



def get_configuration_dict():
    # https://stackoverflow.com/questions/8884188/how-to-read-and-write-ini-file-with-python3
    try:
        from configparser import ConfigParser
    except ImportError:
        from ConfigParser import ConfigParser  # ver. < 3.0

    # instantiate
    config = ConfigParser()

    # parse existing file in the cr_beamng_cosimulation module
    directory_that_contains_me = os.path.dirname(__file__)
    configuration_file = os.path.join(directory_that_contains_me, "beamng.ini")

    # Make sure we have one
    assert os.path.exists(configuration_file), f"Configuration file does not exist: {configuration_file}"

    config.read(configuration_file)

    # read values from a section
    CONFIG_DICT = {}

    CONFIG_DICT["beamng_host"] = config.get('BeamNG', 'host')
    CONFIG_DICT["beamng_port"] = config.getint('BeamNG', 'port')
    CONFIG_DICT["beamng_home_folder"] = config.get('BeamNG', 'home_folder')
    CONFIG_DICT["beamng_user_folder"] = config.get('BeamNG', 'user_folder')
    CONFIG_DICT["beamng_default_z_value"] = config.getfloat('BeamNG', 'default_z_value')
    CONFIG_DICT["beamng_default_level_name"] = config.get('BeamNG', 'default_level_name')


    return CONFIG_DICT

# name = "Accent"
# cmap: mpl.colors.ListedColormap = mpl.colormaps[name]  # type:
# colors: list = cmap.colors

def _get_cmap(n, name='hsv'):
    """Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name."""
    return plt.cm.get_cmap(name, n)

def get_color_dict_for_planning_problems(cr_planning_problem_set: CR_PlanningProblemSet) -> Dict[int, Color]:
    # """
    # Sort the planning problems and return a Color (Float3|Float4 tuple)
    # :param cr_planning_problem_set:
    # :return:
    # """
    
    planning_problems = sorted(cr_planning_problem_set.planning_problem_dict.values(), key= lambda pp : pp.planning_problem_id)

    unique_colors = _get_cmap(len(planning_problems), 'rainbow')

    color_mapping = dict()
    for i, planning_problem in enumerate(planning_problems):
        color_mapping[planning_problem.planning_problem_id] = colors.to_hex(unique_colors(i))
    
    return color_mapping


def is_standing_state(planning_problem: CR_PlanningProblem):
    """ Check whethether the given planning problem initial state had 0 speed, acc, yawrate"""
    initial_state = planning_problem.initial_state
    return initial_state.velocity == 0.0 and initial_state.yaw_rate == 0.0 and initial_state.acceleration == 0.0


def get_center_of_bbox(bbox):
    # TODO This returns a point, maybe we should just return the coordinates as np.array?
    fbl = bbox['front_bottom_left']
    fbr = bbox['front_bottom_right']
    rbr = bbox['rear_bottom_right']
    rbl = bbox['rear_bottom_left']
    a = LineString([[fbl[0], fbl[1]], [rbr[0], rbr[1]]])
    b = LineString([[fbr[0], fbr[1]], [rbl[0], rbl[1]]])
    return a.intersection(b)

# Displacement in the vehicle coordinate system (Rounded values!)
VEHICLES_DELTA: Dict[str, Float2] = {}
# Model -> Lat Displacement, Long Displacement
# positive values: right, up (in the direction of the vehicle)
VEHICLES_DELTA['etk800'] = (0.33, 0.29)
VEHICLES_DELTA['autobello'] = (0.16, 0.25)
VEHICLES_DELTA['hopper'] = (0.55, -0.15)

#### BEAMNG CONFIGURATION AND PARAMETERS
def get_beamng_vehicle_models_configuration() -> Dict[str, Dict[str, float]]:
    """
    Return a configuration object with BeamNG parameters.
    This code reads the file beamng.ini from the default location
    """
    beamng_vehicle_models: Dict[str, Dict[str, float]] = dict()
    # https://www.beamng.com/threads/list-of-beamng-cars-in-real-life.84417/

    # Taken from: https://www.automobiledimension.com/model/bmw/serie-3-touring
    beamng_vehicle_models["car"] = dict()
    beamng_vehicle_models["car"]["beamng_vehicle_model"] = "etk800"
    beamng_vehicle_models["car"]["length"] = 4.713
    beamng_vehicle_models["car"]["width"] = 1.827

    # Taken from: https://en.wikipedia.org/wiki/Fiat_500
    beamng_vehicle_models["mini"] = dict()
    beamng_vehicle_models["mini"]["beamng_vehicle_model"] = "autobello"
    beamng_vehicle_models["mini"]["length"] = 2.970
    beamng_vehicle_models["mini"]["width"] = 1.320

    # Taken from: https://en.wikipedia.org/wiki/Jeep_Wrangler_(TJ)
    beamng_vehicle_models["jeep"] = dict()
    beamng_vehicle_models["jeep"]["beamng_vehicle_model"] = "hopper"
    beamng_vehicle_models["jeep"]["length"] = 3.840
    beamng_vehicle_models["jeep"]["width"] = 1.730

    return beamng_vehicle_models

def get_beamng_configuration() -> OmegaConf:
    # Read BeamNG configuration from the default file (beamng.ini inside cr_beamng_cosimulation)
    # and build a configuration object. The following code is a temporary patch to ensure backward
    # compatibility
    beamng_config_dict = {}
    for key, value in get_configuration_dict().items():
        if key.startswith("beamng_"):
            beamng_config_dict[ key.replace("beamng_", "") ] = value

    # Build the BeamNG configuration object
    beamng_config = DictConfig(beamng_config_dict)
    
    # "default" level for beamng
    beamng_config.beamng_level_name = "tech_ground"
    
    return beamng_config

def remap_position(vehicle_model:str, pos: Float3, angle_deg: float):
    """
    Each vehicle in BeamNG has a custom point used to place the vehicle in the simulation. Different vehicles have
    different positions, because it depends on the underlying 3D model. Those are fixed, we store them here and add
    them as we go

    :param vehicle_model:
    :param pos:
    :return:
    """
    # Displacemente was taken at angle 90
    displacement_angle = -90
    # Compute the rotation angle
    rotation_angle = angle_deg - displacement_angle
    #
    rotation_angle_rad = math.radians(rotation_angle)

    # Long says how much to move following the vector defined by angle_deg
    # Lat says how much to move on the right (or along the perpendicular vector) defined by angle_deg
    lat_displacement, long_displacement = VEHICLES_DELTA[vehicle_model]
    # https://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
    x_displacement = lat_displacement * math.cos(rotation_angle_rad) - long_displacement * math.sin(rotation_angle_rad)
    y_displacement = lat_displacement * math.sin(rotation_angle_rad) + long_displacement * math.cos(rotation_angle_rad)

    # Apply the (rotated) displacement to the current position
    x, y, z = pos

    x1 = x - x_displacement
    # This is plus because the y axis of the car is opposite to the absolute system
    y1 = y - y_displacement

    # Return the adjusted coordinates, keeping z
    return x1, y1, z

def get_state_of_beamng_vehicle_at_time_step(beamng_vehicle_id, time_step: int, bng_scenario_dict: dict) -> CR_State:
    return get_state_of_beamng_vehicles(time_step, bng_scenario_dict)[beamng_vehicle_id]

# Should this be exposed by the scenario handler instead?
def get_state_of_beamng_vehicles(time_step: int, bng_scenario_dict: dict) -> Dict[int, CR_State]:
    """
    Poll each vehicle sensors (state, electricity, damage) and build a CommonRoad state with the data in it
    """
    beamng: BeamNGpy = bng_scenario_dict["beamng"]

    vehicles_dict: Dict[int, Beamng_Vehicle] = bng_scenario_dict["vehicles_dict"]
    imus_dict: Dict[int, AdvancedIMU] = bng_scenario_dict["imus_dict"]

    # Access the currently executed scenario
    beamng_scenario = beamng.scenario.get_current()

    # Update the vehicles state
    beamng_scenario.update()

    

    # Create a state object for each vehicle
    states = {}
    # Report any damage value in case of collision. 
    # TODO: This might be unreliable for low-speed and swipe collisions
    damages = {}

    for vehicle_id, beamng_vehicle in vehicles_dict.items():
        # Pull the data from the vehicle
        beamng_vehicle: Beamng_Vehicle
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
        
        # TODO Add damage as an additional return value...
        damaged = True if beamng_vehicle.sensors['damage']['damage'] > 0.0 else False

        # for key, value in damage.items():
        #     setattr(commonroad_state, key, value)

        # commonroad_state.dmg = dmg
        simulation_time = beamng_vehicle.sensors['timer']['time']
    
        commonroad_state.simulation_time = simulation_time
        
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
        damages[vehicle_id] = damaged

    return states, damages

from commonroad.visualization.util import line_marking_to_linestyle


def map_CR_to_BNG_lane_marking(line_marking: CR_LineMarking, is_yellow: bool = False) -> Tuple[str, float]:
    return {
        CR_LineMarking.DASHED: (
            "line_dashed_short" if not is_yellow else "line_dashed_short_yellow",
            0.25),
        CR_LineMarking.SOLID: (
            "line_white" if not is_yellow else "line_yellow",
            0.25),
        CR_LineMarking.BROAD_DASHED: (
            "line_dashed_short" if not is_yellow else "line_dashed_short_yellow",
            0.5),
        CR_LineMarking.BROAD_SOLID: (
            "line_white" if not is_yellow else "line_yellow",
            0.5),

        CR_LineMarking.CURB: ( "curb", 1.0),
        CR_LineMarking.LOWERED_CURB: ("curb", 0.75),

        CR_LineMarking.DASHED_DASHED: (
            "double_line_dashed_short" if not is_yellow else "double_line_dashed_short_yellow",
            1.0), # Might be too much?
        CR_LineMarking.SOLID_SOLID: (
            "double_line" if not is_yellow else "double_line_yellow",
            1.0), # Might be too much?
        CR_LineMarking.SOLID_DASHED: (
            "line_and_dashed" if not is_yellow else "line_and_dashed_yellow",
            1.0), # Might be too much?
        CR_LineMarking.DASHED_SOLID: (
            "dashed_and_line" if not is_yellow else "dashed_and_line",
            1.0), # Might be too much?
        CR_LineMarking.NO_MARKING: (
            "road_asphalt_light",
            0.25
        ),
        # Maybe a Black Line instead?
        CR_LineMarking.UNKNOWN: (
            None,
            None
        )
    }[line_marking]
