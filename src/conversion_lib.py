import roslib
import sys
import rospy
import numpy as np
import datetime
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import dse_constants

# Compute the 2D rotation matrix from the angle theta
def theta_2_rotm(theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return R


# Compute the 2D rotation matrix from the angle theta
def rotm_2_theta(R):
    theta = np.arctan2(-R[0, 1], R[0, 0])
    return theta


# Converts a quaternion into euler angles, using the euler order described in dse_constants.py
def quat2eul(quat):
    r = R.from_quat(quat)
    eul = r.as_euler(dse_constants.EULER_ORDER)
    return eul


# Converts euler angles into a quaternion, using the euler order described in dse_constants.py
def eul2quat(eul):
    r = R.from_euler(dse_constants.EULER_ORDER, eul[:, 0])
    quat = r.as_quat()
    return quat


# Expects a quaternion in the form: orientation.x,y,z,w
def quat_from_pose2eul(orientation):
    quat = [0, 0, 0, 0]
    quat[0] = orientation.x
    quat[1] = orientation.y
    quat[2] = orientation.z
    quat[3] = orientation.w
    eul = quat2eul(quat)
    return eul


# Expects a quaternion in the form: orientation.x,y,z,w
def euler2quat_from_pose(orientation, euler):
    quat = eul2quat(euler)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation


def state_12D_to_6D(x_12D):
    num_objs = int(len(x_12D) / 12)
    x_6D = np.zeros((num_objs * 6, 1))
    for i in range(num_objs):
        i_6D_low = 6 * i
        i_12D_low = 12 * i
        x_6D[i_6D_low + 0] = x_12D[i_12D_low + 0]
        x_6D[i_6D_low + 1] = x_12D[i_12D_low + 1]
        x_6D[i_6D_low + 2] = x_12D[i_12D_low + 3]
        x_6D[i_6D_low + 3] = x_12D[i_12D_low + 6]
        x_6D[i_6D_low + 4] = x_12D[i_12D_low + 7]
        x_6D[i_6D_low + 5] = x_12D[i_12D_low + 9]
    return x_6D


# Expects a pose in the form: x, y, z, w
def state_from_pose(pose):
    euler_orientation = quat_from_pose2eul(pose.orientation)
    x = np.array([pose.position.x, pose.position.y, pose.position.z, euler_orientation[0], euler_orientation[1], euler_orientation[2]])[:, None]
    return x


# Expects a pose in the form: x, y, z, w
def state_from_pose_3D(pose):
    euler_orientation = quat_from_pose2eul(pose.orientation)
    x = np.array([pose.position.x, pose.position.y, euler_orientation[0]])[:, None]
    return x


# Expects a state in the form: x, y, z, eul_z, eul_y, eul_x
def pose_from_state(x):
    pose = Pose()
    pose.position.x = x[0, 0]
    pose.position.y = x[1, 0]
    pose.position.z = x[2, 0]
    pose.orientation = euler2quat_from_pose(pose.orientation, x[3:6])
    return pose


# Expects a state in the form: x, y, eul_z
def pose_from_state_3D(x):
    pose = Pose()
    pose.position.x = x[0, 0]
    pose.position.y = x[1, 0]
    pose.position.z = 0
    euler_angles = np.array([x[2, 0], 0, 0])[:, None]
    pose.orientation = euler2quat_from_pose(pose.orientation, euler_angles)
    return pose


# Fill and return a pose array with values from the state variable x
def pose_array_from_state(pose_array, x, dim_state, dim_obs):
    num_objs = int(len(x) / dim_state)
    for i in range(num_objs):

        i_low = dim_state * i
        i_high = i_low + dim_obs
        x_i = x[i_low:i_high]

        if dim_state == 6:
            pose_array.poses.append(pose_from_state_3D(x_i))
        else:
            pose_array.poses.append(pose_from_state(x_i))

    return pose_array


# Fill and return a pose array with values from the state variable x
def state_from_pose_array(pose_array, dim_state, dim_obs):
    num_objs = np.shape(pose_array.poses)[0]
    x = np.zeros((num_objs * dim_state, 1))

    for i in range(num_objs):

        i_low = dim_state * i
        i_high = i_low + dim_obs

        if dim_state == 6:
            x[i_low:i_high] = state_from_pose_3D(pose_array.poses[i])
        else:
            x[i_low:i_high] = state_from_pose(pose_array.poses[i])

    return x


# Expects a pose in the form: x, y, z, w
def measurement_from_pose(pose):
    euler_orientation = quat_from_pose2eul(pose.orientation)
    x = np.array([pose.position.x, pose.position.y, pose.position.z, euler_orientation])[:, None]
    return x


# Expects a pose in the form: x, y, z, w
def measurement_from_pose_3D(pose):
    euler_orientation = quat_from_pose2eul(pose.orientation)
    x = np.array([pose.position.x, pose.position.y, euler_orientation[0]])[:, None]
    return x


# Expects a state in the form: x, y, z, eul_z, eul_y, eul_x
def pose_from_measurement(x):
    pose = Pose()
    pose.position.x = x[0, 0]
    pose.position.y = x[1, 0]
    pose.position.z = x[2, 0]
    pose.orientation = euler2quat_from_pose(pose.orientation, x[3:6])
    return pose


# Expects a state in the form: x, y, eul_z
def pose_from_measurement_3D(x):
    pose = Pose()
    pose.position.x = x[0, 0]
    pose.position.y = x[1, 0]
    pose.position.z = 0
    euler_angles = np.array([x[2, 0], 0, 0])[:, None]
    pose.orientation = euler2quat_from_pose(pose.orientation, euler_angles)
    return pose


# Fill and return a pose array with values from the measurement z
def pose_array_from_measurement(pose_array, z, dim_obs):
    num_objs = int(len(z) / dim_obs)
    for i in range(num_objs):

        i_low = dim_obs * i
        i_high = i_low + dim_obs
        x_i = z[i_low:i_high]

        if dim_obs == 3:
            pose_array.poses.append(pose_from_measurement_3D(x_i))
        else:
            pose_array.poses.append(pose_from_measurement(x_i))

    return pose_array


# Grab the relevant chunk from the input matrix
def sub_matrix(matrix, ids, id, size):
    i = np.where(ids == id)[0][0]
    i_min = i * size
    i_max = i_min + size
    return matrix[i_min:i_max, i_min:i_max]


# Fill in a multi-array ROS message type with a 2D input array
def multi_array_2d_input(mat, multi_arr):
    multi_arr.layout.dim.append(MultiArrayDimension())
    multi_arr.layout.dim.append(MultiArrayDimension())
    multi_arr.layout.dim[0].label = 'rows'
    multi_arr.layout.dim[0].size = np.shape(mat)[0]
    multi_arr.layout.dim[0].stride = np.shape(mat)[0]*np.shape(mat)[1]
    multi_arr.layout.dim[1].label = 'cols'
    multi_arr.layout.dim[1].size = np.shape(mat)[1]
    multi_arr.layout.dim[1].stride = np.shape(mat)[1]
    multi_arr.layout.data_offset = 0

    multi_arr.data = mat.flatten()
    return multi_arr


# Grab and return a 2D array from a multi-array ROS message
def multi_array_2d_output(multi_arr):
    arr = np.array(multi_arr.data)
    shape = [multi_arr.layout.dim[0].size, multi_arr.layout.dim[1].size]
    mat = arr.reshape(shape)
    return mat


def state_to_xyzypr(state):
    state = state[:, 0]
    if len(state) == 6:
        output = np.zeros(6)
        output[0:2] = state[0:2]
        output[5] = state[2]
    else:
        output = state[0:6]
    return output


def state_cov_to_covariance_matrix(cov):
    if np.shape(cov)[0] == 6:
        output_3D = np.zeros((6, 6))
        output_3D[0:2, 0:2] = cov[0:2, 0:2]
        output_3D[0:2, 5] = cov[0:2, 2]
        output_3D[5, 0:2] = cov[2, 0:2]
        output_3D[5, 5] = cov[2, 2]
    else:
        output_3D = cov[0:6, 0:6]
    return output_3D


def rotate_covariance_xyzypr_state(cov, state):
    cov_rot = np.zeros((6, 6))
    r = R.from_euler(dse_constants.EULER_ORDER, state[3:6])
    rotm = r.as_matrix()
    cov_pos = cov[0:3, 0:3]
    cov_pos = rotm.dot(cov_pos).dot(rotm.T)
    cov_rot[0:3, 0:3] = cov_pos
    cov_rot[3:6, 3:6] = cov[3:6, 3:6]
    return cov_rot


def covariance_to_ros_covariance(cov):
    ros_cov = np.zeros(36, dtype=np.float64)
    for i in range(6):
        ros_cov[6*i : 6*(i+1)] = cov[i, :]
    return ros_cov
