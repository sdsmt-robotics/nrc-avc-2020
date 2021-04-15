#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from scipy import linalg
import conversion_lib


class AckEkf:

    def __init__(self):

        # Get params from launch file
        # Define vehicle dimensions - wheel radius
        self.r = rospy.get_param('~r', 0.07)
        # Distance between axels
        self.L = rospy.get_param('~l', 0.36)
        self.xf = np.array(rospy.get_param('~inital_state', [0, 0, 0, 0, 0, 0]))

        # GPS reference point and time variable to set GPS_zero to the first measured coordinate
        self.t = 0
        self.gps_zero = [0, 0]
        self.lat_conv = 111320
        self.lon_conv = 79968

        # Vehicle wheel angle
        self.p = 0
        # magnetometer yaw measurement
        self.mag_yaw = 0
        # gyroscope yaw angular velocity measurement
        self.gyro_yaw_dot = 0
        # GPS velocity data
        self.gps_vel_x = 0
        self.gps_vel_y = 0

        # Measured state
        self.z = np.zeros(6)

        # Measurement matrix
        self.H = np.eye(6)

        # Measurement covariance
        gps_std = 0.25
        gps_vel_std = 0.25
        magnetometer_std = 0.1
        gyro_std = 0.1
        self.V = np.diag([gps_std**2, gps_std**2, magnetometer_std,
                          gps_vel_std**2, gps_vel_std**2, gyro_std])

        # Vehicle motion model covariance
        xy_std = 0.05
        yaw_std = 0.05
        xy_vel_std = 0.1
        yaw_dot_std = 0.1
        self.W = np.diag([xy_std**2, xy_std**2, yaw_std**2,
                          xy_vel_std**2, xy_vel_std**2, yaw_dot_std**2])

        # Estimate covariance
        self.P = np.random.randn((6, 6))

        # Estimated state
        self.xp = np.zeros(6)

        # Store last time for dt calculation
        self.last_time = rospy.Time.now().to_sec()

        # Define publishers and subscribers
        # Publishes the current [x, y, theta] state estimate
        self.pose_pub = rospy.Publisher("/EKF/Odometry", Odometry)

        # Computes the current state estimate from the gps data
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        self.gps_vel_sub = rospy.Subscriber("/vel", TwistStamped, self.gps_vel_callback)
        self.magnetometer_yaw_sub = rospy.Subscriber("/magnetic", Float32, self.magnetometer_callback())
        self.gyro_yaw_dot_sub = rospy.Subscriber("/gyro", Float32, self.gyro_callback)

        # Updates the local speed and wheel angle
        # self.velocity_sub = rospy.Subscriber("/speed_current", Float32, self.velocity_callback)
        self.angle_sub = rospy.Subscriber("/target_wheel_angle", Float32, self.angle_callback)

    def gps_callback(self, data):

        # Compute time delta between last measurement
        dt = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()
        dd = self.r * dt

        # Convert the GPS coordinates to meters away from the first recorded coordinate
        if self.t == 0:
            self.t = 1
            self.gps_zero[0] = data.latitude
            self.gps_zero[1] = data.longitude
            self.z = [0, 0, self.p, 0, 0, 0]
            self.lat_conv = 111320
            self.lon_conv = 40075 * np.cos(self.gps_zero[1] * 180.0/np.pi) / 360.0
        else:
            self.z[0] = (data.latitude - self.gps_zero[0]) * self.lat_conv
            self.z[1] = (data.longitude - self.gps_zero[1]) * self.lon_conv
            self.z[2] = self.p
            self.z[3] = self.gps_vel_x
            self.z[4] = self.gps_vel_y
            self.z[5] = self.p

        # compute w from x and y vel
        v_xy = np.sqrt(self.xf[3]**2 + self.xf[4]**2)
        # If it's less than 0.4 m/s, assume it's noise
        if v_xy > 0.4:
            self.w = v_xy
        else:
            self.w = 0

        # Compute step of Kalman filter
        self.w = np.linalg.norm(self.xf[3:5])
        self.xp[0] = self.xf[0] + dd * self.w * np.cos(self.xf[2])
        self.xp[1] = self.xf[1] + dd * self.w * np.sin(self.xf[2])
        self.xp[2] = self.xf[2] + dd * np.tan(self.p) / self.L
        self.xp[3:6] = self.xf[3:6]
        F1 = [1.0, 0.0, -dd * self.w * np.sin(self.xf[2])]
        F2 = [0, 1, dd * self.w * np.cos(self.xf[2])]
        F = np.array([F1, F2, [0, 0, 1]])
        FT = F.T
        pp = np.dot(F, np.dot(self.P, FT)) + self.V
        y = np.dot(self.H, self.z) - np.dot(self.H, self.xp)
        S = np.dot(self.H, np.dot(pp, self. H.T)) + self.W
        SI = linalg.inv(S)
        kal = np.dot(pp, np.dot(self.H.T, SI))
        self.xf = self.xp + np.dot(kal, y)
        self.P = pp - np.dot(kal, np.dot(self.H, pp))

        # Publish the Pose estimate
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose = conversion_lib.pose_from_state_3D(self.xf[:, None])
        cov = conversion_lib.state_cov_to_covariance_matrix(self.P)
        odom.pose.covariance = list(conversion_lib.covariance_to_ros_covariance(cov))
        odom.twist.twist.linear.x = self.xf[3]
        odom.twist.twist.linear.y = self.xf[4]
        odom.twist.twist.linear.z = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.x = self.xf[5]
        cov_dot = self.P
        cov_dot[0:3, 0:3] = cov_dot[3:6, 3:6]
        cov_dot = conversion_lib.state_cov_to_covariance_matrix(cov_dot)
        odom.pose.covariance = list(conversion_lib.covariance_to_ros_covariance(cov_dot))

        self.pose_pub.publish(odom)

    def gps_vel_callback(self, data):
        self.gps_vel_x = data.twist.linear.x
        self.gps_vel_y = data.twist.linear.y
        # v_xy = np.sqrt(x**2 + y**2)
        # if v_xy > 0.4:
        #     self.w = v_xy
        # else:
        #     self.w = 0

    # def velocity_callback(self, data):
    #     # Update w from data
    #     self.w = data.data

    # update the current vehicle wheel angle
    def angle_callback(self, data):
        # Update p from data
        self.p = data.data

    def magnetometer_callback(self, data):
        # Update p from data
        self.mag_yaw = data.data

    def gyro_callback(self, data):
        # Update p from data
        self.gyro_yaw_dot = data.data


def main():
    rospy.init_node('ack_ekf', anonymous=True)
    kalman = AckEkf()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    kalman.output.release()


if __name__ == '__main__':
    main()
