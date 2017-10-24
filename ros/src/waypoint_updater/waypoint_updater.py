#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Add other member variables
        self.waypoints = None
        self.curr_pose = None

        rospy.spin()

    def next_waypoint(self):
        # Make sure waypoints and current position are initialized
        if self.waypoints and self.curr_pose:
        
            # Get coordinates for the car
            car_pos = self.curr_pose
            car_x = car_pos.x
            car_y = car_pos.y

            min_dist = sys.maxsize
            index = 0

            # Find the closest waypoint to current position
            for i, waypoint in enumerate(self.waypoints):
                
                # Get coordinates for the current waypoint
                wp_pos = waypoint.pose
                wp_x = wp_pos.pose.position.x
                wp_y = wp_pos.pose.position.y

                # Calculate distance
                dist = math.sqrt((car_x - wp_x)**2 + (car_y - wp_y)**2)

                    if dist < min_dist:
                        min_dist = dist
                        index = i

            # Add 1 to the index just to make sure the first point is in front of the car
            # TODO: Find a better way to determine if the nearest point is in front of the car
            return index + 1

    def pose_cb(self, msg):
        # Update current position
        self.curr_pose = msg.pose.pose.position
        
        # Find the next waypoint
        next_wp = self.next_waypoint()

        # Store next waypoints
        waypoints = self.waypoints[next_wp: next_wp + LOOKAHEAD_WPS]

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
