#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np
import matplotlib.pyplot as plt
import math
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers       
        self.sub_wp = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)        
        
        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        
        # System-Level Variables        
        self.updateRatePerSec = 10       # update # times per second
        self.system_ready = False
        
        # World-Level Variables        
        self.waypoints = None
        self.num_wp = None
        self.final_Lane = Lane()
        self.final_Lane.header.frame_id = '/world'
        self.tl_wp_idx = None
        
        # Vehicle-Level Variables
        self.pose = None
        self.position = None
        self.orientation = None
        self.yaw = None
        self.linear_velocity = 0
        self.car_wp_idx = None
        
        # Start the process loop
        self.process_loop()
        
    # Tight process loop
    def process_loop(self):
        rate = rospy.Rate(self.updateRatePerSec)
        while not rospy.is_shutdown():
            if self.system_ready:
                self.car_wp_idx = self.next_waypoint()
                self.update_final_waypoints()
                # print "Car WP Idx: ", self.car_wp_idx, " TL WP Idx: ", self.tl_wp_idx, " Yaw: ", self.yaw, " Vel: ", self.linear_velocity, "T_Vel: ", self.final_Lane.waypoints[0].twist.twist.linear.x
                self.publish_final_waypoints()
            else:
                self.check_system()
            rate.sleep()

    # Checks if the system has been received at least one message from each subscriber
    def check_system(self):
        if self.waypoints is not None and self.pose is not None and self.linear_velocity is not None and self.tl_wp_idx is not None:
            self.system_ready = True
            print "System is ready."
    
    # Publishes the final waypoints (with timestamp)    
    def publish_final_waypoints(self):
        self.final_Lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(self.final_Lane)
        
    def pose_cb(self, msg):
        # Get current pose        
        self.pose = msg.pose
        
        # Extract position, orientation, and yaw
        self.position = self.pose.position
        self.orientation = self.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([self.orientation.x,
                                                                   self.orientation.y,
                                                                   self.orientation.z,
                                                                   self.orientation.w])
            
    def velocity_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x
        
    def traffic_cb(self, msg):
        self.tl_wp_idx = msg.data
        
    def waypoints_cb(self, msg):
        # Do only once        
        if self.waypoints is None:
            self.waypoints = msg.waypoints
            self.num_wp = len(self.waypoints)
            
            # Unsubscribe from future waypoint messages (saves on system resources)
            self.sub_wp.unregister()
            self.sub_wp = None
            
    def get_waypoint_position_at(self, wp_idx):
        return self.waypoints[wp_idx].pose.pose.position.x, self.waypoints[wp_idx].pose.pose.position.y
    
    def get_waypoint_velocity(self, waypoints, wp_idx):
        return waypoints[wp_idx].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp_idx, velocity):
        waypoints[wp_idx].twist.twist.linear.x = velocity   
        
    def wp_square_distance_between(self, wp_idx1, wp_idx2):
        # Validate inputs
        if 0 <= wp_idx1 <= wp_idx2 <= len(self.waypoints):       
            dist = 0
            dl = lambda a, b: (a.x-b.x)**2 + (a.y-b.y)**2
            for i in range(wp_idx1, wp_idx2):
                dist += dl(self.waypoints[i].pose.pose.position, self.waypoints[i+1].pose.pose.position)
            return dist
        else:
            return None
            
    def next_waypoint(self):
        prev_dist = 999999999
        curr_dist = prev_dist - 1
        
        # Initialize beginning of search based on last known car waypoint index        
        if self.car_wp_idx is None:
            wp_idx = 0
        else:
            wp_idx = self.car_wp_idx - 5
            
            # Handles boundary condition at waypoint list extremes            
            if wp_idx < 0:
                wp_idx += self.num_wp 
        
        # Search until minimum distance is found        
        while prev_dist > curr_dist:
            prev_dist = curr_dist            
            
            wp_x, wp_y = self.get_waypoint_position_at(wp_idx)
            diff_x = wp_x - self.position.x
            diff_y = wp_y - self.position.y
            
            curr_dist = (diff_x**2) + (diff_y**2)
            wp_idx += 1
            
            # Handles boundary condition at waypoint list extremes             
            if wp_idx >= self.num_wp:
                wp_idx -= self.num_wp
            
        return wp_idx                        
    
    def update_final_waypoints(self):
        global LOOKAHEAD_WPS
        
        wp_end_idx = self.car_wp_idx + LOOKAHEAD_WPS
        
        # Scale down look ahead if at course end (do only 1 loop)
        if wp_end_idx > self.num_wp:
            wp_end_idx = self.num_wp
        
        # Extract desired waypoints
        self.final_Lane.waypoints = copy.deepcopy(self.waypoints[self.car_wp_idx : wp_end_idx])
  
        # If planning to apply the brakes
        if (self.car_wp_idx < self.tl_wp_idx < wp_end_idx) or (wp_end_idx == self.num_wp): 
            num_final_wp = self.tl_wp_idx - self.car_wp_idx
            decel = 0.2
            curr_speed = -decel*10

            distance_to_stop_light2 = self.wp_square_distance_between(self.car_wp_idx, self.tl_wp_idx)
            print("current distance to light: ", distance_to_stop_light2)
            critical_distance2 = 6.


            # assume current speed is the same as the setpoint at the first waypoint
            for i in range(len(self.final_Lane.waypoints)):
                # if the waypoint is before the traffic stop
                if i + self.car_wp_idx < self.tl_wp_idx:
                    self.final_Lane.waypoints[i].twist.twist.linear.x = max(0.,
                                                            min(-2. + 1 * math.sqrt(distance_to_stop_light2),
                                                                11.1))
                else:
                    self.final_Lane.waypoints[i].twist.twist.linear.x = 0  # (distance_to_stop_light2 / stopping_distance2)
                    pass
                    # defaults to the original velocity

            # if curr_distance_to_stoplight2 > critical_distance2:
            #     # Update target velocities in reverse
            #     for i in range(num_final_wp - 1, -1, -1):
            #         curr_distance_to_stoplight2 = self.wp_square_distance_between(self.car_wp_idx + i, self.tl_wp_idx)
            #         if curr_distance_to_stoplight2 < critical_distance2:
            #             self.final_Lane.waypoints[i].twist.twist.linear.x = 0
            #         else:
            #             self.final_Lane.waypoints[i].twist.twist.linear.x = min( max(curr_speed, 0),
            #                                                           self.final_Lane.waypoints[i].twist.twist.linear.x)
            #         curr_speed += decel
            #
            #     for i in range(num_final_wp, LOOKAHEAD_WPS):
            #         self.final_Lane.waypoints[i].twist.twist.linear.x = 0
            #
            # else: # passed critical distance. STOP!
            #     for i in range(LOOKAHEAD_WPS):
            #         self.final_Lane.waypoints[i].twist.twist.linear.x = 0


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
