#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy import linalg
import conversion_lib


class AckPTheta:

    def __init__(self):

        # Get params from launch file
        points_array = rospy.get_param('~points_array', None)
        delay_time = rospy.get_param('~delay_time', 0)
        self.r = rospy.get_param('~r', 0.07)
        self.l = rospy.get_param('~l', 0.36)
        self.k_p = rospy.get_param('~k_p', 10)
        self.robot_d_sq = rospy.get_param('~threshold_dist', 1) ** 2
        self.target_vel = rospy.get_param('~target_vel', 1)
        self.w = self.target_vel
        init_state = rospy.get_param('~inital_state', [0, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(delay_time)

        # Set up waypoint array
        if points_array is None:
            self.points = np.array([[0, 0], [7, 0], [2, -10], [7, -20], [0, -21], [-7, -20], [-6, -10], [-7, 0], [7, 0]])
        else:
            self.points = points_array
        self.current_point = 0
        self.loc = init_state[0:3]

        # Publishes the current [x, y, theta] state estimate
        self.angle_pub = rospy.Publisher("target_wheel_angle", Float32, queue_size=1)
        self.speed_pub = rospy.Publisher("target_velocity", Float32, queue_size=1)

        # Computes the current state estimate from the gps data
        self.ekf_sub = rospy.Subscriber("/EKF/Odometry", Odometry, self.callback)
        self.velocity_sub = rospy.Subscriber("/speed_current", Float32, self.velocity_callback)

    # Returns the angle difference between the current trajectory and the goal, measured CCW from the current trajectory
    def theta_error(self, x, y, t, x_d, y_d):
        t_goal = np.arctan2(y_d - y, x_d - x)
        e = t_goal - t
        ## CRITICAL: ENSURE THAT THE ERROR IS BETWEEN -PI, PI OTHERWISE IT BEHAVES WEIRD
        if e > np.pi:
            e = -np.pi * 2 + e
        elif e < -np.pi:
            e = np.pi * 2 + e
        return e

    def p_ik(self, e):
        return np.arctan2(self.r*self.l*self.k_p*e, self.w)

    def callback(self, data):
        if self.current_point < self.points.shape[0] - 1:
            print('target waypoint: ', self.points[self.current_point + 1])
            d = (self.points[self.current_point + 1][0] - self.loc[0]) ** 2 + \
                (self.points[self.current_point + 1][1] - self.loc[1]) ** 2
            print('distance', np.sqrt(d))
            ## Compute current position based on last time step and measurement
            # From EKF
            self.loc[0] = data.pose.pose.position.x
            self.loc[1] = data.pose.pose.position.y
            eul = conversion_lib.quat_from_pose2eul(data.pose.pose.orientation)
            self.loc[2] = eul[0]

            ## Compute the angle error
            e = self.theta_error(self.loc[0], self.loc[1], self.loc[2],
                                 self.points[self.current_point + 1][0], self.points[self.current_point + 1][1])
            #print('Angle error is: ' + str(e))
            # Compute new wheel angle ans send it to the car
            phi = self.p_ik(e)
            #print('Angle sending to car: ' + str(phi))
            self.angle_pub.publish(phi)
            self.speed_pub.publish(self.target_vel)

            ## Determine if we passed the obstacle
            d = (self.points[self.current_point + 1][0] - self.loc[0]) ** 2 + \
                (self.points[self.current_point + 1][1] - self.loc[1]) ** 2
            if d < self.robot_d_sq:
                self.current_point += 1
        else:
            self.speed_pub.publish(0.00)
            self.angle_pub.publish(0)

    def velocity_callback(self, data):
        # Update w from data
        self.w = data.data
        #print('Speed is: ' + str(self.w))

def main():
    rospy.init_node('ack_p_theta', anonymous=True)
    p_theta = AckPTheta()
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
