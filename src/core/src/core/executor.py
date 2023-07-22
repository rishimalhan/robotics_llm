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


class Executor:
    def __init__(self, group_name="manipulator"):
        self.group_name = group_name
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface(synchronous=True)
        self.move_group = MoveGroupCommander(self.group_name)

    def __del__(self):
        self.scene.clear()
        rospy.sleep(0.2)

    def execute(
        self, trajectory, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0
    ):
        try:
            trajectory = self.move_group.retime_trajectory(
                self.move_group.get_current_state(),
                trajectory,
                velocity_scaling_factor=velocity_scaling_factor,
                acceleration_scaling_factor=acceleration_scaling_factor,
                algorithm="iterative_time_parameterization",
            )
            self.move_group.execute(trajectory, wait=True)
            self.move_group.stop()
        except ValueError as value_exc:
            logger.warn(f"Trajectory execution failed due to: {value_exc}")
        except Exception as exc:
            logger.warn(f"Trajectory execution failed due to: {exc}")
