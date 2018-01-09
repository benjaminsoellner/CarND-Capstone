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

import math

STATE_COUNT_THRESHOLD = 3


# author: udacity, TrW236
class TLDetector(object):


    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.cb_current_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.cb_base_waypoints)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.cb_vehicle_traffic_lights)
        rospy.Subscriber('/image_color', Image, self.cb_image_color)

        config_string = rospy.get_param("/traffic_light_config")
        # TODO what is config string?
        # stored camera_info: w, h = 800, 600 and 8 points as a list
        # rospy.loginfo('config_string: %s', config_string)  # what is config_string
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()


    def cb_current_pose(self, msg):
        """callback for receiving PoseStamped message on /current_pose topic"""
        self.pose = msg


    def cb_base_waypoints(self, waypoints):
        """callback for receiving Lane message on /cb_base_waypoints topic"""
        self.waypoints = waypoints


    def cb_vehicle_traffic_lights(self, msg):
        """callback for receiving TrafficLightArray message on /vehicle/traffic_light topic"""
        self.lights = msg.lights


    def cb_image_color(self, msg):
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
        elif self.state_count >= STATE_COUNT_THRESHOLD:  # threshold=3
            self.last_state = self.state
            if state != TrafficLight.RED:
                light_wp = -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """

        # pose -> point and quaternion, waypoints.waypoints[i].pose.pose is the same as the argument pose
        if self.waypoints is not None:  # search for the wp at the first time
            self.n_wps = len(self.waypoints.waypoints)

            minDist = 1.0e10
            self.minIdx = -1
            for idx, wp in enumerate(self.waypoints.waypoints):
                dist = self.dist_3d(pose, wp.pose.pose)
                if dist < minDist:
                    minDist = dist
                    self.minIdx = idx
            rospy.loginfo('first time: current_pos: %s; closest_wp: %s; closest_idx: %s', pose, minDist, self.minIdx)
            return self.minIdx

        else:
            nextIdx = self.minIdx + 1
            nextIdx %= self.n_wps
            dist1 = self.dist_3d(pose, self.waypoints.waypoints[self.minIdx])
            dist2 = self.dist_3d(pose, self.waypoints.waypoints[nextIdx])
            if dist2 < dist1:
                self.minIdx = nextIdx
            else:
                # self.minIdx = self.minIdx
                pass
            rospy.loginfo('current_pos: %s; closest_idx: %s', pose, self.minIdx)
            return self.minIdx


    def dist_3d(self, pose1, pose2):
        dist_x = (pose1.position.x - pose2.position.x) ** 2
        dist_y = (pose1.position.y - pose2.position.y) ** 2
        dist_z = (pose1.position.z - pose2.position.z) ** 2
        return dist_x + dist_y + dist_z  # no need to calculate the sqrt


    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # Get classification
        # TODO FIXME for test return only RED traffic light
        return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for
        # a given intersection
        stop_line_positions = self.config['stop_line_positions']  # stored 8 points
        # rospy.loginfo('stop_line_position: %s', stop_line_positions)  # what is stop_line_pos
        if (self.pose):
            # car_position on the path (pose.pose -> point and quaternion)
            car_position = self.get_closest_waypoint(self.pose.pose)

        # TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)  # for test return only RED=0 tl state
            return light_wp, state
        # self.waypoints = None   # don't need base_wps by next itr
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
