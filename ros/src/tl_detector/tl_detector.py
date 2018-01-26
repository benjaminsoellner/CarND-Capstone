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

# Number of times a traffic light state needs to be observed until it is deemed reliable
STATE_COUNT_THRESHOLD = 0  # TODO fine-tune

# Distance in meters from which a traffic light can be observed
MAX_TL_DIST = 120  # TODO fine-tune

# Set to "True" if you want to use the simulator traffic light labels
# instead of the classifier
BYPASS_TL_CLASSIFIER = False

# set to true to save the images from the simulator run
SAVE_SIM_IMAGES = False

# Use this constant (!= 1) to throttle the classifier in case of performance issues
CLASSIFY_EVERY_NTH_FRAME = 5


def dist(point1, point2):
    """Helper function to calculate the eucledian distance between two points.
    Accepts both points either in the form (p.x, p.y) or (p[0], p[1])."""
    x1 = None
    x2 = None
    y1 = None
    y2 = None
    if type(point1) is list:
        x1 = point1[0]
        y1 = point1[1]
    else:
        x1 = point1.x
        y1 = point1.y
    if type(point2) is list:
        x2 = point2[0]
        y2 = point2[1]
    else:
        x2 = point2.x
        y2 = point2.y
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def get_orientation(pose_stamped):
    """get orientation from pose"""
    return [
        pose_stamped.orientation.x,
        pose_stamped.orientation.y,
        pose_stamped.orientation.z,
        pose_stamped.orientation.w
    ]


def get_yaw(pose):
    """get orientation from pose_stamped"""
    euler = tf.transformations.euler_from_quaternion(get_orientation(pose))
    return euler[2]  # z direction


def find_closest_point(list_of_points, other_point):
    """finds point that is closest to a list of points"""
    closest_dist = None
    closest_point_index = None
    # iterate through all points ...
    for i in range(len(list_of_points)):
        # ... and calculate the distance to the current position for each
        d = dist(list_of_points[i], other_point)
        # keep track of the closest one
        if closest_dist is None or d < closest_dist:
            closest_dist = d
            closest_point_index = i
    # return it
    return closest_point_index


def find_point_ahead_of_pose(list_of_points, pose):
    """finds index of next waypoint ahead of a pose taking into account the bearing direction.
    Assumes that list_of_points is ordered and any two points are somewhat distant
    so that if the closest point is behind the given pose, the next point will be in front of it."""
    other_point = pose.position
    theta = get_yaw(pose)
    # find closest waypoint first
    closest_point_index = find_closest_point(list_of_points, pose.position)
    if closest_point_index is not None:
        closest_point = list_of_points[closest_point_index]
        closest_x = None
        closest_y = None
        if type(closest_point) is list:
            closest_x = closest_point[0]
            closest_y = closest_point[1]
        else:
            closest_x = closest_point.x
            closest_y = closest_point.y
        # check that closest waypoint is really ahead of me
        angle = math.atan2(closest_y-other_point.y, closest_x-other_point.x)
        diff_angle = math.fabs(angle-theta)
        # if not, the next surely will be (% takes care of rotating list)
        if math.pi * 1.5 > diff_angle > math.pi / 2:
            closest_point_index = (closest_point_index+1) % len(list_of_points)
    return closest_point_index


# author: udacity, TrW236, benjaminsoellner
class TLDetector(object):


    def __init__(self):

        # configuration
        # stored camera_info: w, h = 800, 600 and 8 points as a list
        self.config = yaml.load(rospy.get_param("/traffic_light_config"))
        # self.cfg_camera_info = self.config['camera_info'] # TODO not needed?
        self.cfg_stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        # used for classification
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener() # TODO needed?
        self.image_index = 0

        # telemetry
        self.pose = None # current pose
        self.bare_waypoints = None # waypoints, "bare" means simple list of x/y coordinates
        self.camera_image = None # current camera image
        self.lights = None # current list of traffic lights
        self.light_ahead = None # traffic light index ahead of current pose

        # higher level state
        self.classification_age = 0 # used to count the number of frames since last classification
        self.state = TrafficLight.RED
        self.uncertain_state = TrafficLight.UNKNOWN # uncertain state, kept until occured a couple of times
        self.state_count = 0 # counter to derive certain from uncertain state
        self.target_waypoint = None # closest waypoint to traffic light

        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.cb_current_pose)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.cb_base_waypoints)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.cb_vehicle_traffic_lights)
        rospy.Subscriber('/image_color', Image, self.cb_image_color)

        rospy.spin()



    def cb_base_waypoints(self, waypoints):
        """callback for receiving Lane message on /cb_base_waypoints topic"""
        # only initialize waypoints once (save some ressources)
        if self.bare_waypoints is None:
            self.bare_waypoints = []
            for wp in waypoints.waypoints:
                self.bare_waypoints.append(wp.pose.pose.position)
        # TODO: Unsubscribe to save ressources ?!? - Somehting like self.sub_base_waypoints.unsubscribe()


    def cb_vehicle_traffic_lights(self, msg):
        """callback for receiving TrafficLightArray message on /vehicle/traffic_light topic"""
        # store the traffic light data
        self.lights = msg.lights


    def cb_current_pose(self, msg):
        """callback for receiving PoseStamped message on /current_pose topic"""
        self.pose = msg
        # check that we received all the telemetry
        if self.bare_waypoints is not None:
            # try to find a traffic light ahead of our car
            new_light_ahead = find_point_ahead_of_pose(self.cfg_stop_line_positions, self.pose.pose)
            if new_light_ahead is not None and new_light_ahead != self.light_ahead:
                # we found a new one! save it.
                self.light_ahead = new_light_ahead
                # find waypoint that is closest to the stop line associated with traffic light
                stop_line_ahead = self.cfg_stop_line_positions[self.light_ahead]
                self.target_waypoint = find_closest_point(self.bare_waypoints, stop_line_ahead)
                rospy.logdebug("TLDetector::cb_current_pose found new target waypoint #%s based on new traffic light #%s",
                        self.target_waypoint, self.light_ahead)


    def cb_image_color(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        self.has_image = True
        self.camera_image = msg

        # Write image to file for training with simulation images
        if SAVE_SIM_IMAGES:
            # Save every n-th image (n=10)
            if ((self.image_index % 10) == 0):
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                img_filename = '/home/student/sim_images/' + 'image' + str(self.image_index) + '.jpg'
                rospy.logdebug(img_filename)
                cv2.imwrite(img_filename, cv_image)
            self.image_index += 1

        # check that we are not bypassing the classifier
        if not BYPASS_TL_CLASSIFIER:
            if CLASSIFY_EVERY_NTH_FRAME == self.classification_age+1:
                self.classification_age = 0
                new_state = self.get_state_from_camera()
                # with camera, new state has to occur at least a number of times ...
                if new_state is not False:  # when False -> not close enough
                    rospy.logdebug("TLDetector::cb_image_color classified traffic light #%s with state: %s",
                                   self.light_ahead, self.uncertain_state)
                if self.uncertain_state != new_state:
                    self.state_count = -1
                    self.uncertain_state = new_state
                elif self.state_count == STATE_COUNT_THRESHOLD:
                    # ... before it is set
                    self.state = self.uncertain_state
                    rospy.logdebug("TLDetector::cb_image_color is now sure that traffic light #%s has state: %s",
                            self.light_ahead, self.state)
                if self.state_count < STATE_COUNT_THRESHOLD+1:
                    self.state_count += 1
            else:
                self.classification_age += 1
                # rospy.logdebug("TLDetector::cb_image_color skipping image (%s)", self.classification_age)
        else:
            # bypass classifier if we use simulator data
            new_state = self.get_state_from_simulator()
            # if state changed, save it, log it, and publish it
            if new_state != self.state:
                rospy.logdebug("TLDetector::cb_image_color bypassed classifier. Ground truth for traffic light #%s is: %s",
                        self.light_ahead, self.state)
                self.state = new_state
        # make sure to publish at camera frequency
        self.publish_traffic_waypoint()


    def is_light_close_enough(self):
        """calculates if a light is close enough to current car position in
        order towarrant classification. returns false if no position or traffic
        light is known from telemetry."""
        # pre-conditions - waiting until we have all data from pub-sub
        if self.light_ahead is not None and self.pose is not None:
            d = dist(self.cfg_stop_line_positions[self.light_ahead],
                        self.pose.pose.position)
            return d <= MAX_TL_DIST
        else:
            return False


    def get_state_from_simulator(self):
        # do we have all the telemetry? - if not, assume "UNKNOWN"
        if self.lights is not None and self.light_ahead is not None:
            # are we close enough to the traffic light so that it would be visible?
            if self.is_light_close_enough():
                # return ground truth
                return self.lights[self.light_ahead].state
            else:
                return TrafficLight.UNKNOWN
        else:
            return TrafficLight.UNKNOWN


    def get_state_from_camera(self):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # check that we have all the telemetry - if not, assume "UNKNOWN"
        if self.has_image:
            # are we close enough to the traffic light so that it would be visible?
            if self.is_light_close_enough():
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
                # Get classification
                return self.light_classifier.get_classification(cv_image)
            else:
                return False  # not close enough
        else:
            return TrafficLight.UNKNOWN


    def publish_traffic_waypoint(self):
        # do we have all telemetry? is there a red light?
        if self.target_waypoint is not None and (self.state == TrafficLight.RED or
                                                 self.state == TrafficLight.YELLOW or
                                                 self.state == TrafficLight.UNKNOWN):
            # then publish its waypoint downstream to stop the car
            self.upcoming_red_light_pub.publish(Int32(self.target_waypoint))
        else:
            # otherwise inform downstream systems that road is clear
            self.upcoming_red_light_pub.publish(Int32(-1))


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
