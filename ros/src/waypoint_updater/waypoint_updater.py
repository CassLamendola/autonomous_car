#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Add other member variables
        self.waypoints = None
        self.curr_pose = None
        self.next_traffic_stop_waypoint = -1
        self.current_linear_velocity = None
        self.current_angular_velocity = None

        rospy.spin()

    def next_waypoint(self):
        # Make sure waypoints and current position are initialized
        if self.waypoints and self.curr_pose:

            # Get coordinates for the car
            car_pos = self.curr_pose.position
            car_x = car_pos.x
            car_y = car_pos.y

            car_theta = 2* math.acos(self.curr_pose.orientation.w) if self.curr_pose.orientation.w >= 0 else -2 * math.acos(-self.curr_pose.orientation.w)

            car_heading_x = math.cos(car_theta)
            car_heading_y = math.sin(car_theta)

            min_dist = min_dist_euclid = sys.maxsize
            index = 0
            best_index_euclid = 0

            # Find the closest waypoint to current position
            for i, waypoint in enumerate(self.waypoints):

                # Get coordinates for the current waypoint
                wp_pos = waypoint.pose
                wp_x = wp_pos.pose.position.x
                wp_y = wp_pos.pose.position.y

                # ignore this point if it is behind the current heading of the car
                # get the dot product between the current heading and the
                # waypoint difference
                waypoint_diff_x = wp_x - car_x
                waypoint_diff_y = wp_y - car_y

                car_heading_norm = car_heading_y * car_heading_y + car_heading_x * car_heading_x
                waypoint_diff_norm = waypoint_diff_x * waypoint_diff_x + waypoint_diff_y * waypoint_diff_y

                dot_prod = (car_heading_x * waypoint_diff_x + car_heading_y * waypoint_diff_y)

                # if i == 5020:
                #     print("dot product to 5020: ", dot_prod, "waypoint diff norm: ", waypoint_diff_norm, \
                #           "waypoint_diff_x: ", waypoint_diff_x, "waypoint_diff_y: ", waypoint_diff_y, "car_heading_x: ", car_heading_x, "car_heading_y: ", car_heading_y)
                # if i == 6280:
                #     print("dot product to 6280: ", dot_prod, "waypoint diff norm: ", waypoint_diff_norm)

                # Calculate distance
                dist = waypoint_diff_norm

                if dist < min_dist_euclid:
                    min_dist_euclid = dist
                    best_index_euclid = i

                if dot_prod < 0.:
                    # ignore points that are behind the vehicle
                    continue

                if dist < min_dist:
                    min_dist = dist
                    index = i

            print("best_euclidean: ", best_index_euclid)
            return index

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear
        self.current_angular_velocity = msg.twist.angular

    def pose_cb(self, msg):

        # Update current position
        self.curr_pose = msg.pose

        next_wp = None

        # Find the next waypoint
        next_wp = self.next_waypoint() + 5

        # Store next waypoints
        if next_wp != None:

            next_wp_2 = next_wp_end_2 = next_wp_end = None

            if next_wp + LOOKAHEAD_WPS < len(self.waypoints):
                next_wp_end = next_wp + LOOKAHEAD_WPS
                waypoints = copy.deepcopy(self.waypoints[next_wp: next_wp_end])
                print("lower, next_traffic, upper ", next_wp, self.next_traffic_stop_waypoint, next_wp_end)
            else:
                next_wp_end = len(self.waypoints) - 1
                waypoints = copy.deepcopy(self.waypoints[next_wp: next_wp_end])
                next_wp_2 = 0
                next_wp_end_2 = LOOKAHEAD_WPS - len(waypoints)
                waypoints = copy.deepcopy(self.waypoints[:next_wp_end_2])
                print("lower, next_traffic, upper ", next_wp, self.next_traffic_stop_waypoint, next_wp_end_2)

            # adjust waypoint information to traffic light information
            if self.next_traffic_stop_waypoint != -1 and \
                    (next_wp_end > self.next_traffic_stop_waypoint > next_wp \
                    or next_wp_end_2 > self.next_traffic_stop_waypoint > next_wp_2 \
                     ):

                car_pos = self.curr_pose.position
                car_x = car_pos.x
                car_y = car_pos.y
                distance_to_stop_light2 = (car_x - self.waypoints[self.next_traffic_stop_waypoint].pose.pose.position.x)**2 + \
                                          (car_y - self.waypoints[self.next_traffic_stop_waypoint].pose.pose.position.y)**2

                # assume current speed is the same as the setpoint at the first waypoint
                for i in range(len(waypoints)):
                    # if the waypoint is before the traffic stop
                    if i+next_wp < self.next_traffic_stop_waypoint:
                        waypoints[i].twist.twist.linear.x = max(0, min(-1 +.25*math.sqrt(distance_to_stop_light2), 11.1))
                    else:
                        waypoints[i].twist.twist.linear.x *= 0  # (distance_to_stop_light2 / stopping_distance2)
                        pass
                        # defaults to the original velocity
            else:
                pass
                # The traffic lights are green

            # The snippet below prints the delay in this callback relative to the incoming msg creation time
            now = rospy.get_rostime()
            print("Delay in pose callback: ", (now - msg.header.stamp).to_sec() )
            # Publish waypoints
            message = Lane(waypoints=waypoints)
            self.final_waypoints_pub.publish(message)

    def waypoints_cb(self, lane):
        # Published only once - doesn't change
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.next_traffic_stop_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance2_simple(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[wp2].pose.pose.position)
        return dist



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
