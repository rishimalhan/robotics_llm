#! /usr/bin/python3

from __future__ import absolute_import, division, print_function, unicode_literals

__metaclass__ = type


# External

from moveit_commander.planning_scene_interface import PlanningSceneInterface
import logging

# Internal

from core.utils import get_param, get_absolute_file_path, dict_pose_to_ros_pose
from core.visualization import visualize_planning_scene
from core.planner import Planner
from core.executor import Executor

logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)


def bootstrap_robot():
    trajectory = Planner().plan_to_configuration(configuration=[0, 0, 0, 0, 0, 0])
    Executor().execute(trajectory=trajectory)


def bootstrap_environment():
    """
    Bootstrap the Moveit environment by loading appropriate STLs and Geomtries to planning scene
    """
    environment = get_param("environment")
    meshes = environment.get("meshes")
    scene = PlanningSceneInterface(synchronous=True)
    visual_meshes = []
    if meshes:
        for mesh_name, mesh in meshes.items():
            collision_mesh_path = mesh.get("collision")
            pose = dict_pose_to_ros_pose(mesh.get("pose"))
            visual_meshes.append((get_absolute_file_path(mesh.get("visual")), pose))
            mesh_path = get_absolute_file_path(collision_mesh_path)
            scene.add_mesh(mesh_name, pose, mesh_path)
    visualize_planning_scene(visual_meshes)


def bootstrap(args):
    bootstrap_environment()
    bootstrap_robot()
