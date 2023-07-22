#!/usr/bin/python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

# External

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import logging
import argparse
from IPython import embed
from core.planner import Planner
from core.executor import Executor
from core.utils import dict_grasp_to_target, get_param

# Internal
from core.bootstrap import bootstrap

logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)

parser = argparse.ArgumentParser()
args = parser.parse_args()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("test_moveit", anonymous=True)

bootstrap(args)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

planning_frame = move_group.get_planning_frame()
logger.info(f"Planning frame: {planning_frame}")

eef_link = move_group.get_end_effector_link()
logger.info(f"EE Link: {eef_link}")

logger.info(f"Current robot state: {robot.get_current_state().joint_state.position}")

grasps = get_param("environment").get("grasps")
singulation_planner = Planner()
executor = Executor()
trajectory = singulation_planner.plan_to_target(
    target=dict_grasp_to_target(grasps.get("bin_6"), robot)
)
executor.execute(trajectory)
embed()
