#! /usr/bin/python3

from __future__ import absolute_import, division, print_function, unicode_literals

__metaclass__ = type

# External

import rospy
import copy
from moveit_commander.robot import RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
import logging

# Internal

from core.utils import (
    pose_stamped_to_matrix,
    matrix_to_pose_stamped,
    concatenate_trajectories,
    get_dict_grasp_from_state,
    dict_grasp_to_target,
)

logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)

MAX_CARTESIAN_ATTEMPTS = 5
HOME_TARGET = [0.8, 0, 0.5, 0, 3.14, 0]


class Planner:
    def __init__(self, group_name="manipulator"):
        self.group_name = group_name
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface(synchronous=True)
        self.move_group = MoveGroupCommander(self.group_name)

    def plan_to_home(self):
        dict_target = get_dict_grasp_from_state(HOME_TARGET)
        target = dict_grasp_to_target(dict_target, self.robot)
        trajectory = Planner().plan_to_target(target)
        return trajectory

    def compute_cartesian_path(self, poses, avoid_collisions=True):
        for i in range(MAX_CARTESIAN_ATTEMPTS):
            (trajectory, fraction) = self.move_group.compute_cartesian_path(
                [pose.pose for pose in poses],
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
        return self.retime_path(trajectory)

    def plan_to_target(self, target=None):
        try:
            if target:
                (success, trajectory, planning_time, error_code) = self.move_group.plan(
                    target.pose
                )
            else:
                (
                    success,
                    trajectory,
                    planning_time,
                    error_code,
                ) = self.move_group.plan()
            if success:
                logger.info(
                    f"Planning to target succeeded in duration: {planning_time}"
                )
                return self.retime_path(trajectory)
            else:
                logger.warn(
                    f"Planning to target failed due to moveit error: {error_code}"
                )
        except Exception as exc:
            logger.warn(f"Planning to target failed: {exc}")
        return None

    def plan_to_configuration(self, configuration):
        try:
            self.move_group.set_joint_value_target(configuration)
        except Exception as exc:
            logger.warn(f"Setting joint value target failed: {exc}")
        return self.retime_path(self.plan_to_target())

    def retime_path(
        self,
        path,
        velocity_scaling_factor=1.0,
        acceleration_scaling_factor=1.0,
        reference_state=None,
    ):
        if reference_state is None:
            reference_state = self.move_group.get_current_state()
        trajectory = self.move_group.retime_trajectory(
            reference_state,
            path,
            velocity_scaling_factor=velocity_scaling_factor,
            acceleration_scaling_factor=acceleration_scaling_factor,
            algorithm="iterative_time_parameterization",
        )
        return trajectory

    def pseudo_robot_state(self, trajectory):
        robot_state = self.robot.get_current_state()
        robot_state.joint_state.position = trajectory.joint_trajectory.points[
            -1
        ].positions
        robot_state.joint_state.velocity = trajectory.joint_trajectory.points[
            -1
        ].velocities
        self.move_group.set_start_state(robot_state)

    def plan_grasp(self, target, approach_offset=0.2, scale_factors=0.01):
        # Translate the target away
        approach_matrix = pose_stamped_to_matrix(target)
        approach_matrix[0:3, 3] -= approach_matrix[0:3, 2] * approach_offset
        approach_target = matrix_to_pose_stamped(approach_matrix)
        start_state = copy.deepcopy(self.robot.get_current_state())

        approach_trajectory = self.plan_to_target(approach_target)
        self.pseudo_robot_state(approach_trajectory)

        contact_trajectory = self.compute_cartesian_path([approach_target, target])
        self.pseudo_robot_state(contact_trajectory)
        reverse_trajectory = self.compute_cartesian_path([target, approach_target])
        affordance_trajectory = self.retime_path(
            concatenate_trajectories(contact_trajectory, reverse_trajectory),
            velocity_scaling_factor=scale_factors,
            acceleration_scaling_factor=scale_factors,
            reference_state=self.robot.get_current_state(),
        )
        self.move_group.set_start_state(start_state)

        return [approach_trajectory, affordance_trajectory]


"""
Flavor of the blog post. Tone of post is as if these are new ideas.
Large number of people are beginning to look at it and this post is an illustration
"""
