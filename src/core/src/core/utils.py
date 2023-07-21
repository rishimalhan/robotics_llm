# External

from sensor_msgs.msg import JointState
import numpy
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import (
    quaternion_from_euler,
    quaternion_matrix,
    euler_matrix,
    euler_from_matrix,
    euler_from_quaternion,
)
import rospkg
import roslib
import rosparam
import os


def get_joint_state_msg(state, group_name="joint"):
    config = JointState()
    config.name = "".joint(group_name, "_", [str(i + 1) for i in range(6)])
    config.position = state
    return config


def tf_to_state(tf):
    return numpy.hstack((tf[0], euler_from_quaternion(tf[1], "rxyz")))


def tf_to_matrix(tf):
    matrix = quaternion_matrix([tf[1][0], tf[1][1], tf[1][2], tf[1][3]])
    matrix[0:3, 3] = tf[0]
    return matrix


def tool0_from_camera(state, transformer):
    base_T_camera = state_to_matrix(state)
    tool0_T_camera_vec = transformer.lookupTransform(
        "tool0", "camera_depth_optical_frame", rospy.Time(0)
    )
    tool0_T_camera = quaternion_matrix(
        [
            tool0_T_camera_vec[1][0],
            tool0_T_camera_vec[1][1],
            tool0_T_camera_vec[1][2],
            tool0_T_camera_vec[1][3],
        ]
    )
    tool0_T_camera[0:3, 3] = tool0_T_camera_vec[0]
    return matrix_to_state(
        numpy.matmul(base_T_camera, numpy.linalg.inv(tool0_T_camera))
    )


def pose_to_state(pose):
    return numpy.hstack(
        (
            [pose.position.x, pose.position.y, pose.position.z],
            euler_from_quaternion(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            ),
        )
    )


def state_to_pose(state):
    pose = Pose()
    pose.position.x = state[0]
    pose.position.y = state[1]
    pose.position.z = state[2]
    quaternion = quaternion_from_euler(state[3], state[4], state[5], "rxyz")
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def dict_pose_to_ros_pose(dict_pose):
    pose = PoseStamped()
    if not dict_pose.get("header"):
        pose.header.frame_id = "base_link"
    pose.pose.position.x = dict_pose.get("position").get("x")
    pose.pose.position.y = dict_pose.get("position").get("y")
    pose.pose.position.z = dict_pose.get("position").get("z")
    pose.pose.orientation.x = dict_pose.get("orientation").get("x")
    pose.pose.orientation.y = dict_pose.get("orientation").get("y")
    pose.pose.orientation.z = dict_pose.get("orientation").get("z")
    pose.pose.orientation.w = dict_pose.get("orientation").get("w")
    return pose


def state_to_matrix(state):
    matrix = euler_matrix(state[3], state[4], state[5], "rxyz")
    matrix[0:3, 3] = state[0:3]
    return matrix


def matrix_to_state(matrix):
    return numpy.hstack((matrix[0:3, 3], euler_from_matrix(matrix, "rxyz")))


def get_pkg_path(pkg_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials
    return rospack.get_path(pkg_name)


def load_yaml(pkg_name, yaml_file, namespace=""):
    if namespace:
        namespace = "/" + namespace
    path = get_pkg_path(pkg_name)
    roslib.load_manifest("rosparam")
    paramlist = rosparam.load_file(path + "/config/" + yaml_file + ".yaml" + namespace)
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)


def get_param(param, ns=""):
    return rosparam.get_param("/".join([ns, param]))


def get_absolute_file_path(package_file_path):
    # Remove the 'package://' prefix to get the relative file path
    relative_file_path = package_file_path.replace("package://", "")

    # Split the relative file path into package name and file path components
    package_name, file_path = relative_file_path.split("/", 1)

    package_path = get_pkg_path(package_name)

    # Concatenate the package path with the file path to get the absolute file path
    absolute_file_path = os.path.join(package_path, file_path)

    return absolute_file_path
