#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg


class AckEkf:

    def __init__(self, r=0.07, l=0.36, v1=0.02, v2=0.1):

        # Publishes the current [x, y, theta] state estimate
        self.state_pub = rospy.Publisher("", 'Message Type')

        # Computes the current state estimate from the gps data
        self.gps_sub = rospy.Subscriber("", 'Message Type', self.gps_callback)

        # Updates the local speed and wheel angle
        self.velocity_sub = rospy.Subscriber("", 'Message Type', self.velocity_callback)
        self.angle_sub = rospy.Subscriber("", 'Message Type', self.angle_callback)

        # Store last time for dt calculation
        self.last_time = time.time()

        # Define vehicle dimensions - wheel radius
        self.r = r
        # Distance between axels
        self.L = l

        # Measurement variance
        self.var1 = v1 ** 2
        # Vehicle state variance
        self.var2 = v2 ** 2

        # Vehicle speed
        self.w = 0
        # Vehicle wheel angle
        self.p = 0

        # Measured state
        self.z = np.zeros(3)

        # Measurement matrix
        self.H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
        self.HT = self.H.T

        # Measurement covariance
        self.V = np.eye(3) * self.var1

        # Vehicle covariance
        self.W = np.eye(3) * self.var2

        # Estimate covariance
        self.P = np.zeros((3, 3))

        # Estimated state
        self.xf = np.zeros(3)

        # Predicted state
        self.xp = np.zeros(3)

    def gps_callback(self, data):

        # Compute time delta between last measurement
        dt = time.time() - self.last_time
        self.last_time = time.time()
        dd = self.r * dt

        # Get measurement from GPS data
        self.z = np.array([0, 0, 0])

        # Compute step of Kalman filter
        self.xp[0] = self.xf[0] + dd * self.w * np.cos(self.xf[2])
        self.xp[1] = self.xf[1] + dd * self.w * np.sin(self.xf[2])
        self.xp[2] = self.xf[2] + dd * np.tan(self.p) / self.L
        F1 = [1.0, 0.0, -dd * self.w * np.sin(self.xf[2])]
        F2 = [0, 1, dd * self.w * np.cos(self.xf[2])]
        F = np.array([F1, F2, [0, 0, 1]])
        FT = F.T
        pp = np.dot(F, np.dot(self.P, FT)) + self.V
        y = np.dot(self.H, self.z) - np.dot(self.H, self.xp)
        S = np.dot(self.H, np.dot(pp, self. HT)) + self.W
        SI = linalg.inv(S)
        kal = np.dot(pp, np.dot(self.HT, SI))
        self.xf = self.xp + np.dot(kal, y)
        self.P = pp - np.dot(kal, np.dot(self.H, pp))

        self.state_pub.Publish()

    def velocity_callback(self, data):
        # Update w from data
        self.w = 0

    def angle_callback(self, data):
        # Update p from data
        self.p = 0


def main():
    kalman = AckEkf()
    rospy.init_node('ack_ekf', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    kalman.output.release()


if __name__ == '__main__':
    main()
