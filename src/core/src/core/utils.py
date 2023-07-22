# External

from sensor_msgs.msg import JointState
import numpy as np
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
import tf

# Internal

from core.visualization import visualize_target


def get_joint_state_msg(state, group_name="joint"):
    config = JointState()
    config.name = "".joint(group_name, "_", [str(i + 1) for i in range(6)])
    config.position = state
    return config


def tf_to_state(tf):
    return np.hstack((tf[0], euler_from_quaternion(tf[1], "rxyz")))


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
    return matrix_to_state(np.matmul(base_T_camera, np.linalg.inv(tool0_T_camera)))


def pose_to_state(pose):
    return np.hstack(
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


def pose_stamped_to_matrix(pose):
    pose = pose.pose
    q0 = pose.orientation.x
    q1 = pose.orientation.y
    q2 = pose.orientation.z
    q3 = pose.orientation.w

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    transform = np.eye(4)
    transform[0:3, 0:3] = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
    transform[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

    return transform


def matrix_to_pose_stamped(matrix, frame_id="base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = matrix[0, 0]
    pose.pose.orientation.x = matrix[1, 0]
    pose.pose.orientation.y = matrix[2, 0]
    pose.pose.orientation.z = matrix[0, 1]
    pose.pose.position.x = matrix[0, 3]
    pose.pose.position.y = matrix[1, 3]
    pose.pose.position.z = matrix[2, 3]
    return pose


def transform_pose_to_world(base, pose):
    base_T = pose_stamped_to_matrix(base)
    pose_T = pose_stamped_to_matrix(pose)

    # Multiply T2 with T1 to get the transformed matrix
    transformed_T = np.matmul(base_T, pose_T)

    # Convert the transformed matrix back to PoseStamped
    transformed_pose = matrix_to_pose_stamped(transformed_T, pose.header.frame_id)

    return transformed_pose


def dict_grasp_to_target(dict_grasp, robot, visualize=True):
    base_pose = dict_pose_to_ros_pose(dict_grasp.get("base"))
    grasp_pose = dict_pose_to_ros_pose(dict_grasp.get("pose"))
    world_grasp_pose = transform_pose_to_world(base_pose, grasp_pose)
    world_ee_pose = get_ee_from_pose(world_grasp_pose, robot)
    if visualize:
        br = tf.TransformBroadcaster()
        visualize_target(
            pose_stamped_to_matrix(world_grasp_pose),
            pose_stamped_to_matrix(world_ee_pose),
            br,
        )
    return world_ee_pose


def get_ee_from_pose(pose, robot):
    grasp_frame = robot.get_link("grasp_frame").pose()
    ee_frame = robot.get_link("tool0").pose()
    grasp_matrix = pose_stamped_to_matrix(grasp_frame)
    ee_matrix = pose_stamped_to_matrix(ee_frame)
    grasp_T_ee = np.matmul(np.linalg.inv(grasp_matrix), ee_matrix)
    target_matrix = pose_stamped_to_matrix(pose)
    target_matrix[0:3, 3] -= target_matrix[0:3, 2] * 0.01

    return matrix_to_pose_stamped(np.matmul(target_matrix, grasp_T_ee))


def state_to_matrix(state):
    matrix = euler_matrix(state[3], state[4], state[5], "rxyz")
    matrix[0:3, 3] = state[0:3]
    return matrix


def matrix_to_state(matrix):
    return np.hstack((matrix[0:3, 3], euler_from_matrix(matrix, "rxyz")))


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
