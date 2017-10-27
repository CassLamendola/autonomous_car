
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from pid import PID
from lowpass import LowPassFilter
import numpy as np
from yaw_controller import YawController

class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base = 2.8498
        steer_ratio = 14.8
        max_lat_accel = 3.
        max_steer_angle = 8.

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.00, max_lat_accel, max_steer_angle)
        self.throttle_pid_controller = PID(1,0.01,10)
        self.low_pass_filter = LowPassFilter(.1,1)

    def control(self, linear_setpoint, angular_setpoint, current_linear_velocity,
                    current_angular_velocity, sampling_time, is_dbw_enabled):
        if is_dbw_enabled:

            forward_error = linear_setpoint - current_linear_velocity
            throttle = self.throttle_pid_controller.step(forward_error, sampling_time)
            throttle = self.low_pass_filter.filt(throttle)

            steering = self.yaw_controller.get_steering(linear_setpoint, angular_setpoint, current_linear_velocity)

            print("linear setpoint: ", linear_setpoint)
            print("current velocity: ", current_linear_velocity)
            print("throttle: ", throttle)
            print("angular: ", angular_setpoint)
            print("steering: ", steering)

            return throttle, 0., steering
