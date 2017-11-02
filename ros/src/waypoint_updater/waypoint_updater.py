#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import copy
import tf

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Add other member variables
        self.waypoints = None
        self.curr_pose = None
        self.next_traffic_stop_waypoint = -1

        rospy.spin()

    def next_waypoint(self):
        index = None        
        
        # Make sure waypoints and current position are initialized        
        if self.waypoints and self.curr_pose:

            # Get coordinates for the car
            car_x = self.curr_pose.position.x
            car_y = self.curr_pose.position.y

            #car_theta = 2 * math.asin(self.curr_pose.orientation.z)
            #car_heading_x = math.cos(car_theta)
            #car_heading_y = math.sin(car_theta)

            _, _, yaw = tf.transformations.euler_from_quaternion([self.curr_pose.orientation.x, 
                                                                  self.curr_pose.orientation.y, 
                                                                  self.curr_pose.orientation.z, 
                                                                  self.curr_pose.orientation.w])        
            car_heading_x = math.cos(yaw)
            car_heading_y = math.sin(yaw)

            min_dist = sys.maxsize

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
                
                dot_prod = (car_heading_x * waypoint_diff_x + car_heading_y * waypoint_diff_y)

                if dot_prod <= 0.:
                    # ignore points that are behind the vehicle
                    continue
                
                # Calculate distance
                dist = (waypoint_diff_x * waypoint_diff_x) + (waypoint_diff_y * waypoint_diff_y)

                if dist < min_dist:
                    min_dist = dist
                    index = i

            return index

    def pose_cb(self, msg):

        # Update current position
        self.curr_pose = msg.pose

        # Find the next waypoint
        next_wp = self.next_waypoint()
        #print "NextWP: ", next_wp, "Waypoint Len: ", len(self.waypoints)

        # Store next waypoints
        if next_wp != None:
            next_wp_end = next_wp + LOOKAHEAD_WPS if next_wp + LOOKAHEAD_WPS < len(self.waypoints) else len(self.waypoints)
            waypoints = copy.deepcopy(self.waypoints[next_wp: next_wp_end])
            #print "NextWP_end: ", next_wp_end, "Short Waypoint Len: ", len(waypoints)

            # adjust waypoint information to traffic light information
            if self.next_traffic_stop_waypoint != -1 and self.next_traffic_stop_waypoint < next_wp_end:
                # assume current speed is the same as the setpoint at the first waypoint
                current_speed = waypoints[0].twist.twist.linear.x
                max_deceleration = .1 # m/(s**2)
                stopping_time = current_speed / max_deceleration
                stopping_distance2 = (stopping_time * current_speed)**2 # Use square of the distance to avoid costly calculation

                for i in range(len(waypoints)):
                    # if the waypoint is before the traffic stop
                    if i+next_wp < self.next_traffic_stop_waypoint:
                        distance_to_stop_light2 = self.distance2_simple(self.waypoints, i+next_wp, self.next_traffic_stop_waypoint)
                        if distance_to_stop_light2 <= stopping_distance2:
                            waypoints[i].twist.twist.linear.x *= distance_to_stop_light2 / stopping_distance2
                    else:
                        pass
                        # defaults to the original velocity
            else:
                pass
                # The traffic lights are green

            # The snippet below prints the delay in this callback relative to the incoming msg creation time
            now = rospy.get_rostime()
            # print("Delay in pose callback: ", (now - msg.header.stamp).to_sec() )
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
