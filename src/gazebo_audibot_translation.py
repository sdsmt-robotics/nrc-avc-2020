#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from scipy import linalg
import conversion_lib


class Translator:
    def __init__(self):

        # Get params from launch file
        points_array = rospy.get_param('~points_array', None)
        self.r = rospy.get_param('~r', 0.07)
        self.l = rospy.get_param('~l', 0.36)
        self.k_p = rospy.get_param('~k_p', 50)
        self.robot_d_sq = rospy.get_param('~threshold_dist', 1) ** 2
        self.target_vel = rospy.get_param('~target_vel', 1)
        self.w = self.target_vel
        init_state = rospy.get_param('~inital_state', [0, 0, 0, 0, 0, 0])
        self.car_vel = 0
        # self.p = 1

        # Publishes the current [x, y, theta] state estimate
        self.angle_sub = rospy.Subscriber("target_wheel_angle", Float32, self.steering_cb)
        self.speed_sub = rospy.Subscriber("target_velocity", Float32, self.throttle_cb)
        self.gazebo_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_cb)

        # Publishes the current [x, y, theta] state estimate
        self.angle_pub = rospy.Publisher("steering_cmd", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("throttle_cmd", Float64, queue_size=1)

    def steering_cb(self, msg):
        # Audibot steering ratio
        steering_ratio = 17.3
        max_wheel_angle = np.pi * steering_ratio
        target_angle = msg.data * steering_ratio
        if target_angle > max_wheel_angle:
            target_angle = max_wheel_angle
        elif target_angle < -max_wheel_angle:
            target_angle = -max_wheel_angle
        self.angle_pub.publish(target_angle)
        return

    def gazebo_cb(self, data):
        car_x_vel = data.twist[1].linear.x
        car_y_vel = data.twist[1].linear.y
        self.car_vel = np.sqrt(car_x_vel**2 + car_y_vel**2)

    def throttle_cb(self, msg):
        # target_vel = msg.data
        # new_vel = self.car_vel + (target_vel - self.car_vel) * self.p
        if self.car_vel < msg.data:
            self.speed_pub.publish(0.1)
        else:
            self.speed_pub.publish(0.03)
        return


def main():
    rospy.init_node('gazebo_audibot_translator', anonymous=True)
    thing = Translator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
# current_point stores the index of the current point
# loc = [x, y, theta] current position
# while current_point < len(points):
    # if current_loc > len(points) - 1:
        # if v > (v_min + v_ramp):
            # v = v - v_ramp
    # elif v < (v_min - v_ramp):
        # v = v + v_ramp
    # e = theta_error()
    # w1, w2 = p_ik
    # loc = fk_dt
    # d = sqrt(points(current_point) - loc)
    # if d < robot_d:
        # current_point += 1
