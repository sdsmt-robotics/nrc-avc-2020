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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy import linalg
import conversion_lib


def pub_markers(markers, publisher, type, scalexyz, rgb):
    marker = MarkerArray()
    for i in range(len(markers)):
        waypoint = Marker()
        waypoint.header.frame_id = "/world"
        waypoint.header.stamp = rospy.Time.now()
        waypoint.type = type
        waypoint.id = i
        waypoint.scale.x = scalexyz[0]
        waypoint.scale.y = scalexyz[1]
        waypoint.scale.z = scalexyz[2]
        waypoint.color.a = 1.0
        waypoint.pose.orientation.w = 1.0
        waypoint.color.r = rgb[0]
        waypoint.color.g = rgb[1]
        waypoint.color.b = rgb[2]
        waypoint.pose.position.x = markers[i][0]
        waypoint.pose.position.y = markers[i][1]
        waypoint.pose.position.z = 0
        marker.markers.append(waypoint)
    publisher.publish(marker)


class Translator:
    def __init__(self):
        self.angle_sub = rospy.Subscriber("target_wheel_angle", Float32, self.steering_cb)
        self.speed_sub = rospy.Subscriber("target_velocity", Float32, self.throttle_cb)
        self.gazebo_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_cb)

        self.stanchion_pub = rospy.Publisher("stanchions", MarkerArray, queue_size=1)
        self.stanchions = [[6, -1], [3, -10], [6, -19], [-6, -19], [-6, -1]]
        self.ramp_pub = rospy.Publisher("ramp", MarkerArray, queue_size=1)
        self.ramp = [[0.75, -20.75]]
        self.hoop_pub = rospy.Publisher("hoop", MarkerArray, queue_size=1)
        self.hoop = [[-6+0.76, -10], [-6-0.76, -10]]

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
        pub_markers(self.hoop, self.hoop_pub, 3, [0.2, 0.2, 2], [0, 1, 0])
        pub_markers(self.ramp, self.ramp_pub, 1, [1.5, 1.5, 2], [0, 0, 1])
        pub_markers(self.stanchions, self.stanchion_pub, 3, [0.2, 0.2, 2], [1, 0, 0])

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
