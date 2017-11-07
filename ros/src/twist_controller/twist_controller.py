
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
        max_lat_accel = 1. # 3.
        max_steer_angle = 8.

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.00, max_lat_accel, max_steer_angle)
        #self.throttle_pid_controller = PID(5,0.01,1)
        self.throttle_pid_controller = PID(5, 0., 0.01)
        self.throttle_low_pass_filter = LowPassFilter(.1,1)
        self.steering_pass_filter = LowPassFilter(1,1)

    def control(self, linear_setpoint, angular_setpoint, current_linear_velocity,
                    current_angular_velocity, sampling_time, is_dbw_enabled):
        if is_dbw_enabled:

            forward_error = linear_setpoint - current_linear_velocity
            throttle = self.throttle_pid_controller.step(forward_error, sampling_time)
            # throttle = self.throttle_low_pass_filter.filt(throttle)

            steering = self.yaw_controller.get_steering(linear_setpoint, angular_setpoint, current_linear_velocity)
            steering = self.steering_pass_filter.filt(steering)

            #  apply breaks proportionally to the negative throttle
            brake = 10 * abs(throttle) if throttle < -0. else 0.
            # brake = min(brake,100)

            # if the linear velocity setpoint is low enough, push the breaks to the metal
            # if linear_setpoint < 0.:
            #     brake = max(brake, 100)

            # print("linear setpoint: ", linear_setpoint)
            # print("current velocity: ", current_linear_velocity)
            # print("throttle: ", throttle)
            # print("brake: ", brake)
            # print("angular: ", angular_setpoint)
            # print("steering: ", steering)

            return throttle, brake, steering
