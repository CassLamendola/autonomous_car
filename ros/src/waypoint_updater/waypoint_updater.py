#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

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

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Add other member variables
        self.waypoints = None
        self.curr_pose = None

        rospy.spin()

    def next_waypoint(self):
        # Make sure waypoints and current position are initialized
        if self.waypoints and self.curr_pose:

            # Get coordinates for the car
            car_pos = self.curr_pose.position
            car_x = car_pos.x
            car_y = car_pos.y

            car_theta = 2 * math.asin(self.curr_pose.orientation.z)
            car_heading_x = math.cos(car_theta)
            car_heading_y = math.sin(car_theta)

            min_dist = sys.maxsize
            index = 0

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
                car_heading_norm = math.sqrt(car_heading_norm)
                waypoint_diff_norm = waypoint_diff_x * waypoint_diff_x + waypoint_diff_y * waypoint_diff_y
                waypoint_diff_norm = math.sqrt(waypoint_diff_norm)

                dot_prod = (car_heading_x * waypoint_diff_x + car_heading_y * waypoint_diff_y) / car_heading_norm / waypoint_diff_norm

                if dot_prod < 0.2:
                    # ignore points that are more than 79 degrees away from the current heading of the car
                    continue

                # Calculate distance
                dist = waypoint_diff_norm

                if dist < min_dist:
                    min_dist = dist
                    index = i

            return index

    def pose_cb(self, msg):

        # Update current position
        self.curr_pose = msg.pose

        # Find the next waypoint
        next_wp = self.next_waypoint()

        # Store next waypoints
        if next_wp:
            next_wp_end = next_wp + LOOKAHEAD_WPS if next_wp + LOOKAHEAD_WPS < len(self.waypoints) else len(self.waypoints) - 1
            waypoints = self.waypoints[next_wp: next_wp_end]

            # Publish waypoints
            message = Lane(waypoints=waypoints)
            self.final_waypoints_pub.publish(message)

    def waypoints_cb(self, lane):
        # Published only once - doesn't change
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
