#!/usr/bin/env python3

"""
ROS2 node for semantic tagging of locations using a mock VLM.

Pipeline:
    camera/pose -> VLM labeling -> embedding -> semantic map update

Also publishes visualization markers for RViz.
"""

import json
import threading
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from fake_VLM import mock_label_image, mock_embedding


def dist2d(p1, p2):
    """
    Compute Euclidean distance between two 2D points.

    Args:
        p1 (tuple): (x, y)
        p2 (tuple): (x, y)

    Returns:
        float: Distance
    """
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

COLOR_MAP = {
    'red_bathroom': (1.0, 0.0, 0.0),
    'green_cubicle': (0.0, 1.0, 0.0),
    'blue_shelf': (0.0, 0.0, 1.0),
    'teal_meeting_room': (0.0, 1.0, 1.0),
}

class TaggerNode(Node):
    """
    Node responsible for building a semantic map online.

    Responsibilities:
        - Detect semantic labels using a VLM
        - Associate labels with robot poses
        - Store embeddings
        - Publish visualization markers
    """

    def __init__(self):
        """Initialize node, TF listener, and ROS interfaces."""
        super().__init__('tagger_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.old_pose = None
        self.semantic_map = {}

        self.cam_data_lck = threading.Lock()

        self.semantic_map_file = get_package_share_directory('gl_navigation') + '/config/semantic_map.json'

        self.semantic_map_pub = self.create_publisher(MarkerArray, 'semantic_markers', 10)

        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.cameraCb, 10)

        self.tag_timer = self.create_timer(1.0, self.tagTick)

        self.get_logger().info("[TaggerNode] Ready")

    def getRobotPose(self):
        """
        Get robot pose in map frame using TF.

        Returns:
            tuple or None: (x, y) if available, else None
        """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def cameraCb(self, msg):
        """
        Store latest camera frame.

        Args:
            msg (Image): Incoming camera image
        """
        with self.cam_data_lck:
            self.latest_view = msg

    def getLatestView(self):
        """
        Get latest camera frame safely.

        Returns:
            Image: Latest image message
        """
        with self.cam_data_lck:
            return self.latest_view

    def publishMarkerArray(self, semantic_map):
        """
        Publish semantic markers for visualization in RViz.

        Args:
            semantic_map (dict): Label -> pose mapping
        """
        marker_array = MarkerArray()
        marker_id = 0

        for label, poses in semantic_map.items():
            (r, g, b) = COLOR_MAP.get(label, (0.5, 0.5, 0.5))

            # spherical pose markers
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rclpy.clock.Clock().now().to_msg()
            marker.ns = label
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(poses["pose"][0])
            marker.pose.position.y = float(poses["pose"][1])
            marker.pose.position.z = 0.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

            # label text markers
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = marker.header.stamp
            text_marker.ns = label
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose = marker.pose
            text_marker.pose.position.z += 0.3  # lift text above sphere

            text_marker.scale.z = 0.2  # text height

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = label

            marker_array.markers.append(text_marker)
            marker_id += 1

        self.semantic_map_pub.publish(marker_array)

    def tagTick(self):
        """
        Periodic loop to detect and tag new semantic locations.

        Workflow:
            - Get robot pose
            - Skip if robot hasn't moved enough
            - Query VLM for label
            - Store label, pose, and embedding
            - Save semantic map
        """
        pose = self.getRobotPose()
        if pose is None:
            return

        # visualize the labels on rviz
        self.publishMarkerArray(self.semantic_map)

        if self.old_pose and (dist2d(pose, self.old_pose) < 0.5):
            # To tag only when robot has moved significantly
            return

        self.get_logger().info(f'[TaggerNode] Checking for semantic label at ({pose[0]:.2f}, {pose[1]:.2f})')

        # * Get label from current view
        # latest_view  self.getLatestView()
        # * Send the view to the CLIP-based mock VLM to extract features & get a label
        # label = mock_label_image(latest_view)

        label = mock_label_image(pose[0], pose[1])

        if label is None or label in self.semantic_map:
            return

        self.old_pose = pose

        self.semantic_map[label] = {
            'pose': list(pose),
            'embedding': mock_embedding(label)
        }

        self.get_logger().info(
            f'[TaggerNode] Tagged location ({pose[0]:.2f}, {pose[1]:.2f}) as "{label}"'
        )

        with open(self.semantic_map_file, 'w') as f:
            json.dump(self.semantic_map, f, indent=4)


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    rclpy.spin(TaggerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()