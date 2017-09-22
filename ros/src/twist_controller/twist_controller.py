import math

import pid
import lowpass
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_LINEAR_VELOCITY = 0.0
MAX_LINEAR_VELOCITY = 1.0
THROTTLE_P_VAL = 10.0
THROTTLE_I_VAL = 0.05
THROTTLE_D_VAL = 0.1

class Controller(object):
    def __init__(self, ego, **kwargs):
        self.ego = ego
        self.throttle_pid = pid.PID(THROTTLE_P_VAL, THROTTLE_I_VAL,
                                    THROTTLE_D_VAL, mn=MIN_LINEAR_VELOCITY,
                                    mx=MAX_LINEAR_VELOCITY)

        self.yaw_controller = yaw_controller.YawController(self.ego.wheel_base, self.ego.steer_ratio, self.ego.min_speed, self.ego.max_lat_accel, self.ego.max_steer_angle)

    def control(self, target_linear_velocity, target_angular_velocity, 
                current_linear_velocity, current_angular_velocity,
                dbw_enabled, sample_time, **kwargs):

        throttle = 0.
        brake = 0.
        steering = self.yaw_controller.get_steering(target_linear_velocity,
                                                                               target_angular_velocity,
                                                                               current_linear_velocity)

        # Only update pid controller if Drive By Wire is enabled
        if dbw_enabled:
            throttle_error = target_linear_velocity - current_linear_velocity
            throttle = self.throttle_pid.step(throttle_error, sample_time)
        else:
            self.throttle_pid.reset()

        # Return throttle, brake, steering
        return throttle, brake, steering
