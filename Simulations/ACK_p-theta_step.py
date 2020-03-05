#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D
from scipy import linalg


class AckPTheta:

    def __init__(self, points_array=None, r=0.07, l=0.36, v_max=2, ramp=True, k_p=50, robot_d=1):

        # Publishes the current [x, y, theta] state estimate
        self.angle_pub = rospy.Publisher("turn_angle", Int16)
	self.velocity_pub = rospy.Publisher("speed_set", Float32)

        # Computes the current state estimate from the gps data
        self.ekf_sub = rospy.Subscriber("/EKF/Pose2D", Pose2D, self.callback)

        if points_array is None:
            self.points = np.array([[0, 0], [10, 0]])
        else:
            self.points = points_array

        self.current_point = 0
        self.k_p = k_p
        self.v = v
        self.robot_d_sq = robot_d ** 2
        self.r = r
        self.l = l

    # Returns the angle difference between the current trajectory and the goal, measured CCW from the current trajectory
    def theta_error(self, x, y, t, x_d, y_d):
        t_goal = np.arctan2(y_d - y, x_d - x)
        e = t_goal - t
        ## CRITICAL: ENSURE THAT THE ERROR IS BETWEEN -PI, PI OTHERWISE IT BEHAVES WEIRD
        if e > np.pi:
            e = np.pi * 2 - e
        elif e < -np.pi:
            e = np.pi * 2 + e
        return e

    def p_ik(self, e):
        phi = np.arctan2(self.r*self.l*e, self.v)
        return phi

    def callback(self, data):
        if self.current_point < self.points.shape[0] - 1:
            ## Compute current position based on last time step and measurement
            # From EKF
            loc = [0, 0, 0]

            ## Compute the angle error
            e = self.theta_error(loc[0], loc[1], loc[2], self.points[self.current_point + 1][0], self.points[self.current_point + 1][1])
	    print('\nCurrent Angle Error: ')
	    print(e)
            # Compute new wheel angle ans send it to the car
            phi = self.p_ik(e)
            self.angle_pub.Publish()

            ## Determine if we passed the obstacle
            d = (self.points[self.current_point + 1][0] - loc[0]) ** 2 + \
                (self.points[self.current_point + 1][1] - loc[1]) ** 2
            if d < self.robot_d_sq:
                self.current_point += 1
	else:
	    self.v = 0

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
