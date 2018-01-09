
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
#
# CAM update jan-02-18
#

from pid import PID
from yaw_controller import YawController


# author: udacity, bernhardrode, carstenMIELENZ
class Controller(object):


    def __init__(self, time_step, vehcile_mass, accel_limit,
                 decel_limit, wheel_radius, wheel_base, steer_ratio,
                 min_speed, max_lat_accel, max_steer_angle):

        self.time_step = time_step
        self.vehcile_mass = vehcile_mass
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        # setup PID and YAW controller
        # TODO Note: PID values K, Kd, Ki are draft - to be tuned
        # TODO Note: YAW values min_speed(set by DBW_node) is draft - to be tuned

        self.pid = PID(3, 0.3, 0.6, decel_limit, accel_limit)
        self.yaw = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, velocity, yaw_, current_velocity, dbw_on):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # DBW is enabled -> update P/I/D values
        if dbw_on:

            # throttle calcultion by PID
            # Note: PID considers break and acceleration limits
            cte = velocity - current_velocity
            throttle = self.pid.step(cte, self.time_step)

            # Brake assignment
            if throttle < 0:
                brake = -1.0 * self.vehcile_mass * self.wheel_radius * throttle
                throttle = 0.0
            else:
                brake = 0.0

            # Steering calculation by YAW controller
            # Note: YAW considers steering limits
            steer = self.yaw.get_steering(velocity, yaw_, current_velocity)

        # DBW disabled
        else:
            self.pid.reset()
            throttle = 0.
            brake = 0.
            steer = 0.

        return throttle, brake, steer
