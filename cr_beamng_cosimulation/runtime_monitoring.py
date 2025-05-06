class TrajectoryTrackingError():
    """
    Use instances of this class to measure the errors associated with each tracked trajectory.
    """

    def __init__(self):
        self.planned_trajectories = dict()
        self.recorded_states = dict()
        self.tracking_errors = dict()

    def record_planned_trajectory(self, planned_trajectory):
        initial_state = planned_trajectory.state_list[0]
        time_step_of_planning = initial_state.time_step
        assert time_step_of_planning not in self.planned_trajectories, f"Duplicated planned trajectory at time_step {time_step_of_planning}"
        self.planned_trajectories[time_step_of_planning] = planned_trajectory

    def record_current_state(self, time_step, state):
        assert time_step not in self.recorded_states, f"Duplicated current state at time {time_step}"
        self.recorded_states[time_step] = state

    def compute_error_at_time_step(self, time_step):
        if time_step not in self.tracking_errors:

            assert time_step in self.planned_trajectories, f"Missing state at time {time_step} in planned trajectories"
            assert time_step in self.recorded_states, f"Missing state at time {time_step} in recorded states"

            # Get the trajectory that was computed at time_step
            trajectory_planned_at_time_step = self.planned_trajectories[time_step]
            # Get the first planning state (0 is current or initial)
            first_of_the_planned_state = trajectory_planned_at_time_step.state_list[0]
            # Get the recorded state at the same time_step
            recorded_state_at_time_step = self.recorded_states[time_step]
            # Compute the difference in their positions (Euclideian difference)
            actual_position_x, actual_position_y = recorded_state_at_time_step.position[0], recorded_state_at_time_step.position[1]
            expected_position_x, expected_position_y = first_of_the_planned_state.position[0], first_of_the_planned_state.position[1]
            # Tracking error is the distance between the two points
            # tracking_error = ( (expected_position_x - actual_position_x) ** 2 + (expected_position_y - actual_position_y) ** 2) ** 0.5
            self.tracking_errors[time_step] = [expected_position_x - actual_position_x, expected_position_y - actual_position_y]
            #self.tracking_errors[time_step] = (tracking_error[0]**2 + tracking_error[1]**2)**0.5

        return self.tracking_errors[time_step]

    def compute_last_error(self):
        assert len(self.planned_trajectories) == len(self.recorded_states), "Last error is not defined, missing data!"
        last_recorded_time_step = max(self.planned_trajectories.keys())
        return self.compute_error_at_time_step(last_recorded_time_step)

    def compute_mean_error(self):
        last_recorded_time_step = max(list(self.planned_trajectories.keys()) + list(self.recorded_states.keys()))
        errors = []
        if last_recorded_time_step == 0:
            return self.compute_error_at_time_step(last_recorded_time_step)
        else:
            for time_step in range(0, last_recorded_time_step):
                if time_step in self.planned_trajectories and time_step in self.recorded_states:
                    errors.append(self.compute_error_at_time_step(time_step))
            return sum(errors) / len(errors)

    def compute_error_at_time_step_over_interval(self, time_step, interval):
        """
        Given the trajectory computed at time_step and looking back of interval states, how far the car ended to be?
        We cannot look ahead! This is FROM time_step back
        """
        initial_time_step_to_consider = time_step - interval
        #print(initial_time_step_to_consider)
        if initial_time_step_to_consider <= 0:
            return float('nan')

        # Get the trajectory computed at initial_time_step_to_consider
        assert initial_time_step_to_consider in self.planned_trajectories, f"Missing state at time {initial_time_step_to_consider} in planned trajectories"

        # Get the trajectory that was computed at time_step
        trajectory_planned_at_initial_time_step_to_consider = self.planned_trajectories[initial_time_step_to_consider]
        #print(interval)
        # Get the state that was planned interval steps after initial_time_step_to_consider
        planned_state_after_interval_time_steps = trajectory_planned_at_initial_time_step_to_consider.state_list[interval]

        # Get the actual state corresponding to the planned state (after time_steps)
        assert planned_state_after_interval_time_steps.time_step in self.recorded_states, f"Missing planned state at time {planned_state_after_interval_time_steps.time_step} in recorded states"
        #print(planned_state_after_interval_time_steps.time_step)
        actual_state_after_interval_time_steps = self.recorded_states[planned_state_after_interval_time_steps.time_step]

        # Compute the distance between the positions
        actual_position_x, actual_position_y = actual_state_after_interval_time_steps.position[0], actual_state_after_interval_time_steps.position[1]
        expected_position_x, expected_position_y = planned_state_after_interval_time_steps.position[0], planned_state_after_interval_time_steps.position[1]
        #tracking_error = ( (expected_position_x - actual_position_x) ** 2 + (expected_position_y - actual_position_y) ** 2) ** 0.5
        tracking_error = [expected_position_x - actual_position_x, expected_position_y - actual_position_y]
        return tracking_error
