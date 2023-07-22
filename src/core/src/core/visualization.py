#!/usr/bin/env python3

# External

import rospy
import threading
from visualization_msgs.msg import Marker, MarkerArray
import logging
from tf.transformations import quaternion_from_matrix


logging.basicConfig()
logger = logging.getLogger("rosout")
logger.setLevel(logging.INFO)


class STLPublisherThread(threading.Thread):
    def __init__(self, visual_meshes):
        threading.Thread.__init__(self)
        self.visual_meshes = visual_meshes
        self.daemon = True

    def run(self):
        pub = rospy.Publisher("planning_scene_visual", MarkerArray, queue_size=10)
        rate = rospy.Rate(10)  # 10 Hz
        marker_array = MarkerArray()
        logger.info(
            f"Number of meshes in the planning scene visualization: {len(self.visual_meshes)}"
        )
        id_counter = 0
        for mesh, pose in self.visual_meshes:
            marker = Marker()
            marker.header.frame_id = "base_link"  # Frame ID of the reference frame
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = "file://" + mesh
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose = pose.pose
            marker.color.a = 1.0  # Alpha value (transparency) of the marker
            marker.color.r = 136 / 255  # Red color of the marker
            marker.color.g = 138 / 255  # Green color of the marker
            marker.color.b = 133 / 255  # Blue color of the marker
            marker.id = id_counter
            id_counter += 1
            marker_array.markers.append(marker)

        while not rospy.is_shutdown():
            pub.publish(marker_array)
            rate.sleep()


def visualize_planning_scene(visual_meshes):
    stl_thread = STLPublisherThread(visual_meshes)

    # Start the thread
    logger.info("Starting Planning Scene Visuals")
    stl_thread.start()


def visualize_target(grasp, ee, br, suffix=""):
    for i in range(5):
        try:
            br.sendTransform(
                grasp[0:3, 3],
                quaternion_from_matrix(grasp),  # xyzw
                rospy.Time.now(),
                "grasp_target",
                "base_link",
            )
            rospy.sleep(0.05)
        except Exception as exc:
            logger.warn(f"Sending grasp transform failed due to: {exc}")

        try:
            br.sendTransform(
                ee[0:3, 3],
                quaternion_from_matrix(ee),  # xyzw
                rospy.Time.now(),
                "ee_target",
                "base_link",
            )
            rospy.sleep(0.05)
        except Exception as exc:
            logger.warn(f"Sending ee transform failed due to: {exc}")
