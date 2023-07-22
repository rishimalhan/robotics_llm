#! /usr/bin/python3

from __future__ import absolute_import, division, print_function, unicode_literals

__metaclass__ = type

# External

import rospy
from moveit_commander.robot import RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
import logging

logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)

MAX_CARTESIAN_ATTEMPTS = 5


class Planner:
    def __init__(self, group_name="manipulator"):
        self.group_name = group_name
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface(synchronous=True)
        self.move_group = MoveGroupCommander(self.group_name)

    def __del__(self):
        self.scene.clear()
        rospy.sleep(0.2)

    def compute_cartesian_path(
        self, pose, avoid_collisions=True, async_exec=False, vel_scale=1.0
    ):
        for i in range(MAX_CARTESIAN_ATTEMPTS):
            (trajectory, fraction) = self.move_group.compute_cartesian_path(
                [pose],
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
        return trajectory

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
                return trajectory
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
        return self.plan_to_target()
