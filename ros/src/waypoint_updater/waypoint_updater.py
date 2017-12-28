#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
REFRESH_RATE_IN_HZ = 10


def dl(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


class WaypointUpdater(object):

    def __init__(self):
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

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # Done: Add other member variables you need below
        self.pose = PoseStamped()
        self.final_waypoints = []

        rospy.spin()

    def pose_cb(self, pose):
        rospy.loginfo('WaypointUpdater::pose_cb %s', pose)
        self.pose = pose

    def waypoints_cb(self, lane):
        rospy.loginfo('WaypointUpdater::waypoints_cb %s', lane)
        self.base_waypoints = lane.waypoints

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish(self):
        rospy.loginfo('WaypointUpdater::Publish')
        lane = Lane()
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
