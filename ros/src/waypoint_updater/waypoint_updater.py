#!/usr/bin/env python
import rospy
import tf
import copy
import time
import math
import json

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

# TODO fine-tune lookahead WPS and waypoint refresh rate for performance
LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
REFRESH_RATE_IN_HZ = 10


def calculate_distance_2d(point_a, point_b):
    """Calculate distance"""
    return math.sqrt(
        (point_a[0] - point_b[0]) ** 2 +
        (point_a[1] - point_b[1]) ** 2 +
        (point_a[2] - point_b[2]) ** 2
    )


def get_position(pose_stamped):
    """get position from pose"""
    return [
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z
    ]


def get_orientation(pose_stamped):
    """get orientation from pose"""
    return [
        pose_stamped.orientation.x,
        pose_stamped.orientation.y,
        pose_stamped.orientation.z,
        pose_stamped.orientation.w
    ]


def get_yaw(pose_stamped):
    """get orientation from pose_stamped"""
    orientation = get_orientation(pose_stamped.pose)

    euler = tf.transformations.euler_from_quaternion(orientation)
    return euler[2]  # z direction


# author: bernhardrode
class WaypointUpdater(object):
    """WaypointUpdater Class"""


    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        rospy.logdebug('WaypointUpdater::__init__ (enter)')
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.cb_current_pose)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints',
                    Lane, self.cb_base_waypoints)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        # below
        rospy.Subscriber('/traffic_waypoint', Int32, self.cb_traffic_waypoint)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.cb_obstacle_waypoint)

        # Create /final_waypoints publisher
        self.pub_final_waypoints = rospy.Publisher('final_waypoints',
                                            Lane, queue_size=1)

        # Member variables
        self.pose_stamped = PoseStamped()
        self.base_waypoints = None
        self.final_waypoints = []

        self.update_logger_time()
        self.loop()


    def loop(self):
        rospy.logdebug('WaypointUpdater::loop (enter)')
        rate = rospy.Rate(REFRESH_RATE_IN_HZ)
        while not rospy.is_shutdown():
            if (self.pose_stamped is not None) and (self.base_waypoints is not None):
                self.update_final_waypoints()
                # self.publish_cte()
                # self.update_velocity(self.final_waypoints) # <xg>: update the
                # velocity
                self.publish_final_waypoints()

                if time.time() > self.logger_time + 1:
                    self.update_logger_time()
            rate.sleep()
        rospy.spin()


    def update_logger_time(self):
        self.logger_time = time.time()


    def cb_current_pose(self, pose_stamped):
        """callback for receiving PoseStamped message on /current_pose topic"""
        rospy.logdebug('WaypointUpdater::pose_cb %s', pose_stamped)
        self.pose_stamped = pose_stamped  # current pos


    def cb_base_waypoints(self, msg):
        """callback for receiving Lane message on /base_waypoints topic"""
        rospy.logdebug('WaypointUpdater::waypoints_cb %s', msg)
        self.base_waypoints = msg.waypoints
        # TODO: Unsubscribe to save ressources ?!? - Somehting like self.sub_base_waypoints.unsubscribe()


    def cb_traffic_waypoint(self, msg):
        """callback for receiving Int32 message on /traffic_waypoint topic."""
        rospy.logdebug('WaypointUpdater::traffic_cb %s', msg)
        # TODO: implement traffic_cb(...) after traffic light detection is implemented


    def cb_obstacle_waypoint(self, msg):
        """callback for receiving Waypoint message on /obstacle_waypoint topic."""
        rospy.logdebug('WaypointUpdater::obstacle_cb %s', msg)
        # TODO: implement obstacle_cb(...) after traffic light detection is implemented


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, index, velocity):
        waypoints[index].twist.twist.linear.x = velocity


    def norm_index(self, index):
        """if index is greater than the length of the waypoint list, wraps it around with modulo operation"""
        rospy.logdebug('WaypointUpdater::norm_index')
        wp_count = len(self.base_waypoints)
        index = abs(index % wp_count)
        return index


    def next_waypoint(self, position, theta):  # theta is yaw
        """finds index of next waypoint ahead of position looking at steering angle theta"""
        rospy.logdebug('WaypointUpdater::next_waypoint')
        # find closest waypoint first
        index = self.closest_waypoint(position)
        map_coords = get_position(self.base_waypoints[index].pose)
        # get coordination of closest waypoints
        map_x = map_coords[0]
        map_y = map_coords[1]
        # check that closest waypoint is really ahead of me
        heading = math.atan2(map_y - position[1], map_x - position[0])
        angle = math.fabs(theta - heading)
        # if not, the next surely will be
        if angle > math.pi / 4:
            index += 1
        # if waypoint index exceeds length of list, wrap around to the start of list
        index = self.norm_index(index)
        return index


    def closest_waypoint(self, position):
        rospy.logdebug('WaypointUpdater::closest_waypoint (enter)')
        closest_len = 10000
        closest_index = 0
        # iterate through all waypoints ...
        for i in range(len(self.base_waypoints)):
            # ... and calculate the distance to the current position for each
            dist = calculate_distance_2d(
                    position, get_position(self.base_waypoints[i].pose) )
            # keep track of the closest one
            if dist < closest_len and dist >= 0:
                closest_len = dist
                closest_index = i
        # return it
        return closest_index


    def update_final_waypoints(self):
        """update the list of waypoints to conclude only the LOOKAHEAD_WPS number of waypoints"""
        rospy.logdebug('WaypointUpdater::update_final_waypoints (enter)')
        theta = get_yaw(self.pose_stamped)
        next_wp_index = self.next_waypoint(get_position(self.pose_stamped), theta)
        # start with an empty list to publish
        final_waypoints = []
        len_base_waypoints = len(self.base_waypoints)
        # fill the empty list only with LOOKAHEAD_WPS number of points
        for i in range(LOOKAHEAD_WPS):
            # add all the waypoints but keep track of cyclicity of list
            tmp_index = (i + next_wp_index) % len_base_waypoints
            # TODO: verify that waypoints already contain correct velocity at this point (should be implemented by traffic_cb / obstacle_cb)
            waypoint = copy.deepcopy(self.base_waypoints[tmp_index])
            final_waypoints.append(waypoint)
        self.final_waypoints = final_waypoints


    def publish_final_waypoints(self):
        rospy.logdebug('WaypointUpdater::publish_final_waypoints (enter)')
        lane = Lane()
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.pub_final_waypoints.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
