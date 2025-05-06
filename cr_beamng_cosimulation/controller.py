import numpy as np
# TODO Is this the right class? or should we have some sort of Curvilinear/Cartesian Trajectory?
from commonroad.scenario.trajectory import Trajectory as CR_Trajectory
from commonroad.scenario.scenario import Scenario as CR_Scenario
from commonroad.scenario.state import State as CR_State
from commonroad.planning.planning_problem import PlanningProblem as CR_PlanningProblem
from vehiclemodels.vehicle_parameters import VehicleParameters
from vehiclemodels.utils.acceleration_constraints import acceleration_constraints
from vehiclemodels.utils.steering_constraints import steering_constraints

from beamngpy.types import Float3


class BngController():
    # TODO @Korbinian, @Marc, and @Yuan this is the core component to implement

    def __init__(self): #, Scenario: CR_Scenario, planning_problem: CR_PlanningProblem):
        # TODO Add/change parameters as you like
        pass

    def generate_control_commands(self,
                                  target_state: CR_State,
                                  vehicle_model: VehicleParameters) -> Float3:
        """
        # TODO Add parameters as you like
        Given a CommonRoad trajectory compute the appropriate control commands (throttle_intensity, brake_intensity, steering)
        throttle_intensity [0, 1]
        brake_intensity = [0,1 ]
        steering = [-1. +1],

        :param trajectory:
        :return: control_commands (throttle_intensity, brake_intensity, steering)
        """

        target_state = target_state.state_list[1]

        # steering
        wheel_base = vehicle_model.a + vehicle_model.b
        steering_min = vehicle_model.steering.min
        steering_max = vehicle_model.steering.max
        # TODO: Note that Alessio commented this out for the moment to make the overall co-simulation run
        # @Korbinian @Marcus: how do we get kappa? Is this the rotation speed in the Frenet frame?
        #steering_angle = np.arctan2(wheel_base * current_state.kappa, 1.0)
        steering_angle = -target_state.steering_angle
        if steering_angle > steering_max:
            steering = 1
        elif steering_angle < steering_min:
            steering = -1
        elif steering_angle >= 0:
            # normalize by max_steering_angle to get value between 0 and 1
            steering = steering_angle/steering_max
        else: # steering_angle < 0
            # normalize by abs(min_steering_angle) to get value between 0 and -1
            steering = steering_angle/abs(steering_min)

        # throttle / brake
        a_max = vehicle_model.longitudinal.a_max

        a = acceleration_constraints(target_state.velocity, target_state.acceleration, vehicle_model.longitudinal)
        print(f"acceleration in controller is {a}")
        if a > 0:
            throttle = a/a_max
            brake = 0
        else:
            throttle = 0
            brake = -a/a_max

        #a = [acceleration_constraints(state.velocity, state.acceleration, vehicle_model.longitudinal) for state in trajectory.state_list]

        # TODO is trajectory from timestep 0 or timestep 1?
        return throttle, brake, steering
        # raise NotImplementedError("This method is not yet implemented!")
