#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                            SteeringCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(kp = 1., ki = 1., kd=1.)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Initialiaze setpoint values and controller sampling values
        self.linear_setpoint = None
        self.angular_setpoint = None
        self.current_linear_velocity = None
        self.current_angular_velocity = None
        self.previous_timestamp = None
        self.current_timestamp = None
        self.is_dbw_enabled = True
        self.is_initialized = False
        self.is_current_velocity_initialized = False
        self.is_velocity_setpoint_initialized = False

        self.loop()

    # Update dbw_enabled status
    def dbw_enabled_cb(self, msg):
        self.is_dbw_enabled = msg.data

    # Update the current commanded vehicle motion rate setpoints
    def twist_cmd_cb(self, msg):
        self.previous_timestamp = self.current_timestamp
        self.current_timestamp = msg.header.stamp
        self.linear_setpoint = msg.twist.linear
        self.angular_setpoint = msg.twist.angular

        # Once a previous timestamp has been received, set the DBWNode as initialized
        if self.previous_timestamp is not None:
            self.is_velocity_setpoint_initialized = True

        if self.is_current_velocity_initialized:
            self.is_initialized = True

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear
        self.current_angular_velocity = msg.twist.angular
        if self.is_velocity_setpoint_initialized:
            self.is_initialized = True

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            # if not yet initialized continue the loop
            if not self.is_initialized or not self.is_dbw_enabled :
                continue

            sampling_time = self.current_timestamp - self.previous_timestamp

            throttle, brake, steering = self.controller.control(self.linear_setpoint,
                                                                self.angular_setpoint,
                                                                self.current_linear_velocity,
                                                                self.current_angular_velocity, # TODO: set current velocity
                                                                sampling_time.to_sec(),
                                                                self.is_dbw_enabled)
            if self.is_dbw_enabled:
                self.publish(throttle, brake, steering)
                print(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
