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


class WaypointUpdater(object):
    """WaypointUpdater Class"""

    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        rospy.loginfo('WaypointUpdater::__init__')
        rospy.init_node('waypoint_updater')

        self.sub_current_pose = rospy.Subscriber(
            '/current_pose', PoseStamped, self.pose_cb)
        self.sub_base_waypoints = rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb)

        # Done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        # below
        self.sub_traffic_waypoint = rospy.Subscriber(
            '/traffic_waypoint', Int32, self.traffic_cb)
        self.sub_obstacle_waypoint = rospy.Subscriber(
            '/obstacle_waypoint', Waypoint, self.obstacle_cb)

        # Create final_waypoints publisher
        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # Member variables
        self.pose_stamped = PoseStamped()
        self.base_waypoints = None
        self.final_waypoints = []

        self.update_logger_time()
        self.loop()

    def loop(self):
        rospy.loginfo('WaypointUpdater::loop')
        rate = rospy.Rate(REFRESH_RATE_IN_HZ)
        while not rospy.is_shutdown():
            if (self.pose_stamped is not None) and (self.base_waypoints is not None):
                self.update_final_waypoints()
                # self.publish_cte()
                # self.update_velocity(self.final_waypoints) # <xg>: update the velocity
                self.publish_final_waypoints()

                if time.time() > self.logger_time + 1:
                    self.update_logger_time()
            rate.sleep()
        rospy.spin()

    def update_logger_time(self):
        self.logger_time = time.time()

    def pose_cb(self, pose_stamped):
        rospy.loginfo('WaypointUpdater::pose_cb %s', pose_stamped)
        self.pose_stamped = pose_stamped

    def waypoints_cb(self, msg):
        rospy.loginfo('WaypointUpdater::waypoints_cb %s', msg)
        self.base_waypoints = msg.waypoints
        # Unsubscribe to save ressources ?!?
        # Somehting like self.sub_base_waypoints.unsubscribe()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. We will implement it
        # later
        rospy.loginfo('WaypointUpdater::traffic_cb %s', msg)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it
        # later
        rospy.loginfo('WaypointUpdater::obstacle_cb %s', msg)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def norm_index(self, index):
        rospy.loginfo('WaypointUpdater::norm_index')
        wp_count = len(self.base_waypoints)
        index = abs(index % wp_count)
        return index

    def next_waypoint(self, position, theta):
        rospy.loginfo('WaypointUpdater::next_waypoint')
        index = self.closest_waypoint(position)
        map_coords = get_position(self.base_waypoints[index].pose)

        map_x = map_coords[0]
        map_y = map_coords[1]

        heading = math.atan2(map_y - position[1], map_x - position[0])
        angle = math.fabs(theta - heading)
        if angle > math.pi / 4:
            index += 1
        index = self.norm_index(index)

        return index

    def closest_waypoint(self, position):
        rospy.loginfo('WaypointUpdater::closest_waypoint')
        closest_len = 10000
        closest_index = 0
        for i in range(len(self.base_waypoints)):
            dist = calculate_distance_2d(
                position,
                get_position(self.base_waypoints[i].pose)
            )
            if dist < closest_len and dist >= 0:
                closest_len = dist
                closest_index = i

        return closest_index

    def update_final_waypoints(self):
        rospy.loginfo('WaypointUpdater::update_final_waypoints')
        theta = get_yaw(self.pose_stamped)
        index = self.next_waypoint(get_position(self.pose_stamped), theta)

        final_waypoints = []
        len_base_waypoints = len(self.base_waypoints)
        for i in range(LOOKAHEAD_WPS):
            tmp_index = (i + index) % len_base_waypoints
            waypoint = copy.deepcopy(self.base_waypoints[tmp_index])
            final_waypoints.append(waypoint)
        self.final_waypoints = final_waypoints

    def publish_final_waypoints(self):
        rospy.loginfo('WaypointUpdater::publish_final_waypoints')
        lane = Lane()
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
