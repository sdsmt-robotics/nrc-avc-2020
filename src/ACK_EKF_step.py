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
        self.xf = np.array(rospy.get_param('~inital_state', [0, 0, 0]))

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
        self.gps_x = 0
        self.gps_y = 0

        # Vehicle wheel angle
        self.p = 0
        # Vehicle forward velocity
        self.w = 0
        # magnetometer yaw measurement
        self.mag_yaw = 0
        self.mag_zero = 0

        # Measured state
        self.z = np.zeros(3)

        # Measurement matrix
        self.H = None
        self.H_all = np.eye(3)
        self.H_gps = np.zeros(3, 2)
        self.H_imu[0, 0] = 1
        self.H_imu[1, 1] = 1
        self.H_imu = np.zeros(3, 1)
        self.H_imu[2, 0] = 1
        self.H_none = np.eye(3)

        # Measurement covariance
        gps_std = 0.25
        # gps_vel_std = 0.25
        magnetometer_std = 0.01
        self.R = None
        self.R_all = np.diag([gps_std**2, gps_std**2, magnetometer_std])
        self.R_gps = np.diag([gps_std**2, gps_std**2])
        self.R_imu = np.diag([magnetometer_std])
        self.R_none = np.diag([gps_std**2, gps_std**2, magnetometer_std])

        # Vehicle motion model covariance
        xy_std = 0.05
        yaw_std = 0.05
        self.Q = np.diag([xy_std**2, xy_std**2, yaw_std**2])

        # Estimate covariance
        self.P = np.diag([xy_std**2, xy_std**2, yaw_std**2])

        # Estimated state
        self.xp = np.zeros(8)

        # Store last time for dt calculation
        self.last_time = rospy.Time.now().to_sec()
        self.last_imu_time = rospy.Time.now().to_sec()
        self.last_gps_time = rospy.Time.now().to_sec()

        time.sleep(5)
        # Define publishers and subscribers
        # Publishes the current [x, y, theta] state estimate
        self.pose_pub = rospy.Publisher("/EKF/Odometry", Odometry, queue_size=1)

        # Computes the current state estimate from the gps data
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

        # Updates the local speed and wheel angle
        self.velocity_sub = rospy.Subscriber("/speed_current", Float32, self.velocity_callback)
        self.angle_sub = rospy.Subscriber("/wheel_angle_current", Float32, self.angle_callback, queue_size=1)

    def kalman_update(self):

        # Compute time delta between last measurement
        dt = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()
        dd = self.r * dt

        # Check sensor last measurement times
        if self.last_gps_time - self.last_time < self.sensor_timeout:
            if self.last_imu_time - self.last_time < self.sensor_timeout:
                # We have data from both sensors
                self.H = self.H_all
                self.z = np.zeros(3)
                self.z[0] = self.gps_x
                self.z[1] = self.gps_y
                self.R = self.R_all
                self.mag_yaw -= self.mag_zero
                if np.abs(self.mag_yaw - self.xf[2]) < np.pi:
                    self.z[2] = self.mag_yaw
                else:
                    if self.mag_yaw - self.xf[2] > np.pi:
                        self.z[2] = self.mag_yaw - 2 * np.pi
                    else:
                        self.z[2] = self.mag_yaw + 2 * np.pi
            else:
                # We only have GPS data
                self.H = self.H_gps
                self.z = np.zeros(2)
                self.z[0] = self.gps_x
                self.z[1] = self.gps_y
                self.R = self.R_gps
        elif self.last_imu_time - self.last_time < self.sensor_timeout:
            # We only have IMU data
            self.H = self.H_imu
            self.z = []
            self.R = self.R_imu
            self.mag_yaw -= self.mag_zero
            if np.abs(self.mag_yaw - self.xf[2]) < np.pi:
                self.z[0] = self.mag_yaw
            else:
                if self.mag_yaw - self.xf[2] > np.pi:
                    self.z[0] = self.mag_yaw - 2 * np.pi
                else:
                    self.z[0] = self.mag_yaw + 2 * np.pi
        else:
            print('no sensor data')
            # We have no sensor data
            self.H = self.H_none
            self.z = None
            self.R = self.R_none

        if self.last_gps_time - self.last_time > self.sensor_timeout:
            print('bad GPS data')

        # Compute one step of Kalman filter
        # State prediction
        self.xp[0] = self.xf[0] + dt * self.w * np.cos(self.xf[2])
        self.xp[1] = self.xf[1] + dt * self.w * np.sin(self.xf[2])
        self.xp[2] = self.xf[2] + dt * self.w * np.tan(self.p) / self.L

        # Compute the motion jacobian H
        F1 = [1, 0, -dt * self.w * np.sin(self.xf[2])]
        F2 = [0, 1, dt * self.w * np.cos(self.xf[2])]
        F = np.array([F1, F2, [0, 0, 1]])

        if self.z is None:
            # We had no measurements, just do the motion update
            pp = np.dot(F, np.dot(self.P, F.T)) + self.Q
            self.xf = self.xp
            self.P = pp
        else:
            # We have measurements, compute the filter normally
            pp = np.dot(F, np.dot(self.P, F.T)) + self.Q
            y = self.z - np.dot(self.H, self.xp)
            S = np.dot(self.H, np.dot(pp, self.H.T)) + self.R
            SI = linalg.inv(S)
            kal = np.dot(pp, np.dot(self.H.T, SI))
            self.xf = self.xp + np.dot(kal, y)
            self.P = pp - np.dot(kal, np.dot(self.H, pp))

        # Wrap the angle between -pi, pi
        while self.xf[2] > np.pi:
            self.xf[2] -= 2*np.pi
        while self.xf[2] < -np.pi:
            self.xf[2] += 2*np.pi

        # Publish the Pose estimate
        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose = conversion_lib.pose_from_state_3D(self.xf[:, None])
        cov = conversion_lib.state_cov_to_covariance_matrix(self.P)
        odom.pose.covariance = list(conversion_lib.covariance_to_ros_covariance(cov))
        self.pose_pub.publish(odom)

    def gps_callback(self, data):
        self.last_gps_time = rospy.Time.now().to_sec()
        # Convert the GPS coordinates to meters away from the first recorded coordinate
        if self.t == 0:
            self.t = 1
            self.gps_zero[0] = data.latitude
            self.gps_zero[1] = data.longitude
            self.gps_x = 0
            self.gps_y = 0
        else:
            self.gps_x = (data.latitude - self.gps_zero[0]) * self.lat_conv
            self.gps_y = (data.longitude - self.gps_zero[1]) * self.lon_conv
            # print(self.gps_x, self.gps_y)

    # W is the vehicle's forward velocity in m/s
    def velocity_callback(self, data):
        # Update w from data
        self.w = data.data

    # update the current vehicle wheel angle
    def angle_callback(self, data):
        # Update p from data
        self.p = data.data

    def imu_cb(self, msg):
        # print('got imu data')
        self.last_imu_time = rospy.Time.now().to_sec()
        self.mag_yaw = conversion_lib.quat_from_pose2eul(msg.orientation)[0]


def main():
    rospy.init_node('ack_ekf', anonymous=True)
    kalman = AckEkf()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
