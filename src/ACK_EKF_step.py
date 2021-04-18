#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from scipy import linalg
import conversion_lib


class AckEkf:

    def __init__(self):

        # Get params from launch file
        self.gps_offset_angle = rospy.get_param('~gps_offset_angle', 0)
        self.gps_rotm = conversion_lib.theta_2_rotm(self.gps_offset_angle)
        self.update_frequency = rospy.get_param('~frequency', 10)
        self.sensor_timeout = rospy.get_param('~sensor_timeout', 2.0)
        # Define vehicle dimensions - wheel radius
        self.r = rospy.get_param('~r', 0.07)
        # Distance between axels
        self.L = rospy.get_param('~l', 0.36)
        self.xf = np.array(rospy.get_param('~inital_state', [0, 0, 0, 0, 0, 0, 0, 0]))

        # # Get params from launch file
        # self.update_frequency = 10
        # self.sensor_timeout = 2.0
        # # Define vehicle dimensions - wheel radius
        # self.r = 0.07
        # # Distance between axels
        # self.L = 0.36
        # self.xf = [0, 0, 0, 0, 0, 0, 0, 0]

        # GPS reference point and time variable to set GPS_zero to the first measured coordinate
        self.t = 0
        self.gps_zero = [0, 0]
        self.lat_conv = 111320
        self.lon_conv = 79968
        # self.re = 6368000 / 20

        # Vehicle wheel angle
        self.p = 0
        # magnetometer yaw measurement
        self.mag_yaw = 0
        self.mag_zero = 0
        # gyroscope yaw angular velocity measurement
        self.gyro_yaw_dot = 0
        # # GPS velocity data
        # self.gps_vel_x = 0
        # self.gps_vel_y = 0
        self.acc_x = 0
        self.acc_y = 0

        # Measured state
        self.z = np.zeros(6)

        # Measurement matrix
        self.H = np.zeros((6, 8))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1
        self.H[3, 5] = 1
        self.H[4, 6] = 1
        self.H[5, 7] = 1

        # Measurement covariance
        gps_std = 0.5
        # gps_vel_std = 0.25
        magnetometer_std = 0.1
        gyro_std = 0.25
        acc_xy_std = 0.25
        self.R = np.diag([gps_std**2, gps_std**2, magnetometer_std,
                          gyro_std, acc_xy_std**2, acc_xy_std**2])

        # Vehicle motion model covariance
        xy_std = 0.05
        yaw_std = 0.05
        xy_vel_std = 0.1
        yaw_dot_std = 0.1
        acc_xy_std = 0.1
        self.Q = np.diag([xy_std**2, xy_std**2, yaw_std**2,
                          xy_vel_std**2, xy_vel_std**2, yaw_dot_std**2,
                          acc_xy_std**2, acc_xy_std**2])

        # Estimate covariance
        self.P = np.diag([xy_std**2, xy_std**2, yaw_std**2,
                          xy_vel_std**2, xy_vel_std**2, yaw_dot_std**2,
                          acc_xy_std**2, acc_xy_std**2])

        # Estimated state
        self.xp = np.zeros(8)

        # Store last time for dt calculation
        self.last_time = rospy.Time.now().to_sec()
        self.last_imu_time = rospy.Time.now().to_sec()

        time.sleep(5)
        # Define publishers and subscribers
        # Publishes the current [x, y, theta] state estimate
        self.pose_pub = rospy.Publisher("/EKF/Odometry", Odometry, queue_size=1)

        # Computes the current state estimate from the gps data
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        # self.gps_vel_sub = rospy.Subscriber("/vel", Vector3Stamped, self.gps_vel_callback)
        # self.magnetometer_yaw_sub = rospy.Subscriber("/magnetic", Float32, self.magnetometer_callback)
        # self.gyro_yaw_dot_sub = rospy.Subscriber("/gyro", Float32, self.gyro_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

        # Updates the local speed and wheel angle
        # self.velocity_sub = rospy.Subscriber("/speed_current", Float32, self.velocity_callback)
        self.angle_sub = rospy.Subscriber("/target_wheel_angle", Float32, self.angle_callback, queue_size=1)

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
            self.mag_zero = self.mag_yaw - self.gps_offset_angle
            # print(self.mag_zero)
            self.z = [0, 0, self.p, 0, 0, 0]
            # self.cos_lat_0 = np.cos(data.latitude)
            # self.x_0 = self.re * data.longitude * self.cos_lat_0
            # self.y_0 = self.re * data.latitude
            self.lat_conv = 111320
            self.lon_conv = 40075 * np.cos(self.gps_zero[1]) * 5 / 2.25
        else:
            # self.x = self.re * data.longitude * self.cos_lat_0 - self.x_0
            # self.y = self.re * data.latitude - self.y_0
            # self.z[0] = self.x
            # self.z[1] = self.y
            gps_xy = np.array([[(data.latitude - self.gps_zero[0]) * self.lat_conv], [(data.longitude - self.gps_zero[1]) * self.lon_conv]])
            self.z[0:2] = self.gps_rotm.dot(gps_xy)
            # print(gps_xy, self.z[0:2])
            # self.z[0] = (data.latitude - self.gps_zero[0]) * self.lat_conv
            # self.z[1] = (data.longitude - self.gps_zero[1]) * self.lon_conv
            self.mag_yaw -= self.mag_zero
            # print('gps deltas: ', self.z[0], self.z[1])
            if np.abs(self.mag_yaw - self.xf[2]) < np.pi:
                self.z[2] = self.mag_yaw
            else:
                if self.mag_yaw - self.xf[2] > np.pi:
                    self.z[2] = self.mag_yaw - 2*np.pi
                else:
                    self.z[2] = self.mag_yaw + 2*np.pi
            # print(np.abs(self.mag_yaw - self.xf[2]))
            self.z[3] = self.gyro_yaw_dot
            self.z[4] = self.acc_x
            self.z[5] = self.acc_y

        # compute w from x and y vel
        # v_xy = np.sqrt(self.xf[3]**2 + self.xf[4]**2)
        # If it's less than 0.4 m/s, assume it's noise
        # if v_xy > 0.4:
        #     self.w = v_xy
        # else:
        #     self.w = 0

        # Compute step of Kalman filter
        self.w = np.linalg.norm(self.xf[3:5])
        # print(self.xf)
        self.xp[0] = self.xf[0] + dd * self.w * np.cos(self.xf[2])
        self.xp[1] = self.xf[1] + dd * self.w * np.sin(self.xf[2])
        self.xp[2] = self.xf[2] + dd * np.tan(self.p) / self.L
        self.xp[3] = self.xf[3] + dt * self.xf[6]
        self.xp[4] = self.xf[4] + dt * self.xf[7]
        self.xp[5:8] = self.xf[5:8]
        F = np.eye(8)
        F[0, :] = [1, 0, -dd * self.w * np.sin(self.xf[2]), 0, 0, 0, 0, 0]
        F[1, :] = [0, 1, dd * self.w * np.cos(self.xf[2]), 0, 0, 0, 0, 0]
        F[3, 6] = dt
        F[4, 7] = dt
        # F = np.array([F1, F2, [0, 0, 1]])
        FT = F.T
        pp = np.dot(F, np.dot(self.P, FT)) + self.Q
        y = self.z - np.dot(self.H, self.xp)
        S = np.dot(self.H, np.dot(pp, self. H.T)) + self.R
        SI = linalg.inv(S)
        kal = np.dot(pp, np.dot(self.H.T, SI))
        self.xf = self.xp + np.dot(kal, y)
        self.P = pp - np.dot(kal, np.dot(self.H, pp))
        while self.xf[2] > np.pi:
            self.xf[2] -= 2*np.pi
        while self.xf[2] < -np.pi:
            self.xf[2] += 2*np.pi

        # Publish the Pose estimate
        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose = conversion_lib.pose_from_state_3D(self.xf[0:6, None])
        cov = conversion_lib.state_cov_to_covariance_matrix(self.P[0:6, 0:6])
        odom.pose.covariance = list(conversion_lib.covariance_to_ros_covariance(cov))
        odom.twist.twist.linear.x = self.xf[3]
        odom.twist.twist.linear.y = self.xf[4]
        odom.twist.twist.linear.z = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.x = self.xf[5]
        cov_dot = self.P[0:6, 0:6]
        cov_dot[0:3, 0:3] = cov_dot[3:6, 3:6]
        cov_dot = conversion_lib.state_cov_to_covariance_matrix(cov_dot)
        odom.twist.covariance = list(conversion_lib.covariance_to_ros_covariance(cov_dot))

        self.pose_pub.publish(odom)

    # def gps_vel_callback(self, data):
    #     self.gps_vel_x = data.vector.x
    #     self.gps_vel_y = data.vector.y
    #     # v_xy = np.sqrt(x**2 + y**2)
    #     # if v_xy > 0.4:
    #     #     self.w = v_xy
    #     # else:
    #     #     self.w = 0

    # def velocity_callback(self, data):
    #     # Update w from data
    #     self.w = data.data

    # update the current vehicle wheel angle
    def angle_callback(self, data):
        # Update p from data
        self.p = data.data * 1.1

    def imu_cb(self, msg):
        self.mag_yaw = conversion_lib.quat_from_pose2eul(msg.orientation)[0]
        self.gyro_yaw_dot = msg.angular_velocity.z
        dt = rospy.Time.now().to_sec() - self.last_imu_time
        self.last_imu_time = rospy.Time.now().to_sec()
        self.acc_x = self.acc_x + msg.linear_acceleration.x * dt
        self.acc_y = self.acc_y + msg.linear_acceleration.y * dt


def main():
    rospy.init_node('ack_ekf', anonymous=True)
    kalman = AckEkf()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
