#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

#
# CAM - import twist_controller
#
from twist_controller import Controller



'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


# author: udacity, carstenMIELENZ, bernhardrode, TrW236
class DBWNode(object):


    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # DONE: Create `TwistController` object

        # node publishes actuation messages
        self.pub_steer = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.pub_throttle = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.pub_brake = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # DONE: Subscribe to all the topics you need to

        # Subscribe to topics according schematic
        rospy.Subscriber('/twist_cmd',           TwistStamped, self.cb_twist_cmd)
        rospy.Subscriber('/current_velocity',    TwistStamped, self.cb_current_velocity)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.cb_vehicle_dbw_enabled)
        rospy.Subscriber('/vehicle/force_brake', Bool,         self.cb_force_brake)

        # Setup import twist_controller
        # TODO Note: min_speed is set to 0.4 which is draft - to be tuned
        self.controller = Controller(1.0/50, vehicle_mass,
                                          accel_limit, decel_limit,
                                          wheel_radius, wheel_base, steer_ratio,
                                          0.4, max_lat_accel, max_steer_angle)

        # Member functions
        self.twist            = None
        self.velocity         = None
        self.yaw              = None
        self.dbw              = False
        self.current_twist    = None
        self.current_velocity = None
        self.force_brake = False

        self.loop()


    # Callbacks for subscribtions
    def cb_force_brake(self, msg):
        self.force_brake = bool(msg.data)


    def cb_twist_cmd(self,msg):
        """callback for receiving TwistStamped message on /twist_cmd topic"""
        # log message
        # rospy.logdebug('DBWNode::twist_cmd_cb %s',msg)
        # store message
        self.twist    = msg.twist
        self.velocity = msg.twist.linear.x
        self.yaw      = msg.twist.angular.z


    def cb_current_velocity(self,msg):
        """callback for receiving TwistStamped message on /current_velocity topic"""
        # log message
        # rospy.logdebug('DBWNode::velocity_cb %s',msg)
        # store message
        self.current_twist    = msg.twist
        self.current_velocity = msg.twist.linear.x


    def cb_vehicle_dbw_enabled(self,msg):
        """callback for receiving Bool message on /vehicle/dbw_enabled topic"""
        # log message
        # rospy.logdebug('DBWNode::dbw_enabled_cb %s',msg)
        # store message
        self.dbw = bool(msg.data)


    def loop(self):
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():

            # Done: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            #

            if self.dbw and self.force_brake:
                self.publish(0, 10000, 0)
                rate.sleep()  # important
                continue

            # Check that input values are ok
            if self.dbw and self.twist is not None and self.current_twist is not None:
                # Get throttle, break, steer by controller
                throttle, brake, steer = self.controller.control(self.velocity,
                                        self.yaw, self.current_velocity, self.dbw)
                # Publish throttle, break, steer values
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.pub_throttle.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.pub_steer.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.pub_brake.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
