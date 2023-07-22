#! /usr/bin/python3

from __future__ import absolute_import, division, print_function, unicode_literals

__metaclass__ = type

# External

import rospy
from moveit_commander.robot import RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
from geometry_msgs.msg import Pose
import numpy as np
import logging
from tf.transformations import quaternion_matrix, quaternion_from_matrix

# Internal

from utils import load_yaml
from utils import state_to_pose

logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)


class MoveitInterface:
    def __init__(self, group_name):
        self.group_name = group_name
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface(synchronous=True)
        self.move_group = MoveGroupCommander(self.group_name)

    def __del__(self):
        self.scene.clear()
        rospy.sleep(0.2)

    def execute_cartesian_path(
        self, waypoints, avoid_collisions=True, async_exec=False, vel_scale=1.0
    ):
        for i in range(10):
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,
                eef_step=0.01,
                jump_threshold=0.0,
                avoid_collisions=avoid_collisions,
            )
            if fraction == 1.0:
                break
        if fraction != 1.0:
            logger.warn(
                "Cartesian planning failure. Only covered {0} fraction of path.".format(
                    fraction
                )
            )
            return None
        if not async_exec:
            self.move_group.execute(
                self.move_group.retime_trajectory(
                    self.move_group.get_current_state(),
                    plan,
                    velocity_scaling_factor=vel_scale,
                ),
                wait=True,
            )
            self.move_group.stop()
        else:
            self.move_group.execute(
                self.move_group.retime_trajectory(
                    self.move_group.get_current_state(),
                    plan,
                    velocity_scaling_factor=vel_scale,
                ),
                wait=False,
            )
        return plan

    def execute(self, goal, async_exec=False, vel_scale=1.0):
        for i in range(5):
            (error_flag, plan, planning_time, error_code) = self.move_group.plan(goal)
            if error_flag:
                break
        if error_flag:
            logger.info(
                "Planning successful. Planning time: {0} s. Executing trajectory".format(
                    planning_time
                )
            )
        else:
            logger.warning(error_code)
            return
        if not async_exec:
            self.move_group.execute(
                self.move_group.retime_trajectory(
                    self.move_group.get_current_state(),
                    plan,
                    velocity_scaling_factor=vel_scale,
                ),
                wait=True,
            )
            self.move_group.stop()
        else:
            self.move_group.execute(
                self.move_group.retime_trajectory(
                    self.move_group.get_current_state(),
                    plan,
                    velocity_scaling_factor=vel_scale,
                ),
                wait=False,
            )
        return plan

    def get_current_forward_kinematics(self):
        current_pose = self.move_group.get_current_pose().pose
        forward_kinematics = quaternion_matrix(
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
        )
        forward_kinematics[0:3, 3] = [
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
        ]
        return numpy.array(forward_kinematics)

    def pose_error(self, joints):
        import IPython

        IPython.embed()
        fk = self.get_fk.get_fk(joints)

    def execute_joint_path(self, joint_states, camera):
        for i, joint_state in enumerate(joint_states):
            logger.info("Trajectory point: %d", i + 1)
            self.execute(joint_state, vel_scale=0.01)
        return True


def bootstrap_system(sim_camera=False):
    # Bootstrap the robot parameters
    load_yaml("system", "system")
    inspection_bot = InspectionBot()
    return inspection_bot
