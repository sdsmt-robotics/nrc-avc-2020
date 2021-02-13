#!/usr/bin/env python
import rospy
import roslib
import sys
import time
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from scipy import linalg


class TestDriving:

    def __init__(self, points_array=None, r=0.07, l=0.36, k_p=50, robot_d=1):

        # Publishes the current [x, y, theta] state estimate
        self.angle_pub = rospy.Publisher("turn_angle", Int16)
        self.speed_pub = rospy.Publisher("speed_set", Float32)
	self.k = 0
        self.velocity = np.concatenate((np.arange(50), np.arange(50)[::-1])) / 25.0
        self.turn_angle = np.concatenate((np.arange(50), np.arange(50)[::-1])) + (90 - 25)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
	    self.k = (self.k + 1) % 100
	    self.angle_pub.publish(self.turn_angle[self.k])
	    self.speed_pub.publish(self.velocity[self.k])
            rate.sleep()

def main():
    rospy.init_node('test_driving_node', anonymous=True)
    drive = TestDriving()
    try:
        drive.run()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
