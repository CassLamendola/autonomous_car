#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
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

LOOKAHEAD_WPS = 105 # Number of waypoints we will publish. You can change this number

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
                print "\nCar WP Idx: ", self.car_wp_idx, " TL WP Idx: ", self.tl_wp_idx, " Yaw: ", self.yaw, " Vel: ", self.linear_velocity, "T_Vel: ", self.final_Lane.waypoints[0].twist.twist.linear.x
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
        
        # Scale down look ahead if at course end (does only 1 loop)
        if wp_end_idx > self.num_wp:
            wp_end_idx = self.num_wp
        
        # Extract desired waypoints
        self.final_Lane.waypoints = copy.deepcopy(self.waypoints[self.car_wp_idx : wp_end_idx])
    
        # Step-based braking profile. Allows car to creep up to the stop line.
        if (self.car_wp_idx < self.tl_wp_idx < wp_end_idx) or (wp_end_idx == self.num_wp):
            tl_offset = 15
            idle_speed_mps = 3 
            num_final_wp = wp_end_idx - self.car_wp_idx
            threshold_idx = self.tl_wp_idx - tl_offset - self.car_wp_idx
            
            # Update target velocities           
            for i in range(0, num_final_wp - 1):
                if (i > threshold_idx):
                    self.final_Lane.waypoints[i].twist.twist.linear.x = 0
                else:
                    self.final_Lane.waypoints[i].twist.twist.linear.x =  idle_speed_mps
  
           
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
