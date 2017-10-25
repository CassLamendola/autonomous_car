
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from pid import PID
import numpy as np

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.linear_pid_controller = PID(**kwargs)
        self.angular_pid_controller = PID(**kwargs)

    def control(self, linear_setpoint, angular_setpoint, current_linear_velocity,
                    current_angular_velocity, sampling_time, is_dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if is_dbw_enabled:
            linear_setpoint = np.array([linear_setpoint.x, linear_setpoint.y, linear_setpoint.z])
            current_linear_velocity = np.array([current_linear_velocity.x, current_linear_velocity.y, current_linear_velocity.z])
            # angular_setpoint = np.array([angular_setpoint.x, angular_setpoint.y, angular_setpoint.z])
            # current_angular_velocity = np.array([current_angular_velocity.x, current_angular_velocity.y, current_angular_velocity.z])

            forward_error = np.linalg.norm(linear_setpoint-current_linear_velocity)
            angular_error = angular_setpoint.z-current_angular_velocity.z

            print("forward erorr: ", forward_error, " angular_error: ", angular_error)

            throttle = self.linear_pid_controller.step(forward_error, sampling_time)
            steering = self.angular_pid_controller.step(angular_error, sampling_time)
            return forward_error/100, 0., -angular_error
