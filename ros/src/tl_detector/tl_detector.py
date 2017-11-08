#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import sys
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_index_to_pose(self, list_of_poses, pose):
        """Get the index of the pose in list_of_poses that is closest to pose, that isn't already behind the heading
            of pose

        Args:
            list_of_poses: A list of poses
            pose: The pose to query against the list of poses

        Returns:
            int: index of the pose in list_of_poses that is closest to pose

        """
        car_x = pose.position.x
        car_y = pose.position.y
        _, _, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])        
        car_heading_x = math.cos(yaw)
        car_heading_y = math.sin(yaw)

        min_dist = sys.maxsize
        index = None
        
        #print "X: ", car_x, " Y: ", car_y, " Yaw: ", yaw, " HeadX: " , car_heading_x, " HeadY: ", car_heading_y

        # Find the closest waypoint to current position
        for i, waypoint in enumerate(list_of_poses):

            # Get coordinates for the current waypoint
            wp_pos = waypoint.pose
            wp_x = wp_pos.pose.position.x
            wp_y = wp_pos.pose.position.y

            # ignore this point if it is behind the current heading of the car
            # get the dot product between the current heading and the
            # waypoint difference
            waypoint_diff_x = wp_x - car_x
            waypoint_diff_y = wp_y - car_y

            dot_prod = car_heading_x * waypoint_diff_x + car_heading_y * waypoint_diff_y
            
            if dot_prod < 0:
                # ignore points that are behind the car's heading
                continue

            # Calculate distance        
            dist = (waypoint_diff_x * waypoint_diff_x) + (waypoint_diff_y * waypoint_diff_y)

            if dist < min_dist:
                #print "dist: ", dist, "min_dist:", min_dist
                min_dist = dist
                index = i

        return index

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        index = None

        if self.waypoints:
            index = self.get_closest_index_to_pose(self.waypoints.waypoints, pose)

        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_closest_traffic_light_to_waypoint(self, waypoint_index):
        """Get the closest traffic light index to the given wapoint index

        Args:
            waypoint_index: index of the waypoint

        Returns:
            int: index of the traffic light that is closest to the waypoint_index

        """
        index = None

        
        #if self.lights != None:        
            #print "Total Lights: ", len(self.lights)

#        if self.waypoints != None:       
            #print "Total Waypoints: ", len(self.waypoints.waypoints)
        
        if self.lights and self.waypoints:
            index = self.get_closest_index_to_pose(self.lights, self.waypoints.waypoints[waypoint_index].pose.pose)

        return index

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_state = TrafficLight.UNKNOWN
        stop_waypoint_index = -1

        # return -1, TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #print "car_position: ", car_position
            
            #TODO find the closest visible traffic light (if one exists)
            light_index = self.get_closest_traffic_light_to_waypoint(car_position)
            #print " light_index: ", light_index
            
            if light_index != None:
                light = self.lights[light_index]
                ###light_state = light.state
		light_state = self.get_light_state(light) ###
                stop_light_pose=Pose()
                stop_light_pose.position.x=stop_line_positions[light_index][0]
                stop_light_pose.position.y=stop_line_positions[light_index][1]
                stop_waypoint_index = self.get_closest_waypoint(stop_light_pose)
            else:
                self.waypoints = None

        print "stop waypoint index: ", stop_waypoint_index, "light state: ", light_state
        
        return stop_waypoint_index, light_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
