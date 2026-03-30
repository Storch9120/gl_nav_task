#!/usr/bin/env python3

"""
Occupancy Grid Publisher Node

This node:
- Loads a map image (.png) and corresponding YAML metadata
- Converts the image into a binary occupancy grid
- Publishes it periodically on the /map topic

Occupancy encoding:
    0   -> free
    100 -> occupied
    -1  -> unknown (not used here)
"""

import os
import yaml
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from ament_index_python.packages import get_package_share_directory


class OccupancyGridPublisher(Node):
    """
    ROS2 node that converts a grayscale image into an OccupancyGrid.

    Parameters:
        image_file (str): Path to map image
        threshold (int): Pixel threshold for occupancy classification
    """

    def __init__(self):
        """Initialize node, load map parameters, and start publisher."""
        super().__init__('occupancy_grid_publisher')

        pkg_gl_navigation = get_package_share_directory('gl_navigation')
        default_map_file = os.path.join(pkg_gl_navigation, 'config', 'small_room_map.png')

        self.declare_parameter('image_file', default_map_file)
        self.declare_parameter('threshold', 200)  # alpha<threshold = occupied

        self.map_file = self.get_parameter('image_file').value
        self.map_params_file = self.map_file.replace('.png', '.yaml') # ! assuming no troll names like pngABC.png
        with open(self.map_params_file, 'r') as f:
            params = yaml.safe_load(f)
            self.resolution = params['resolution']
            self.origin_x = params['origin'][0]
            self.origin_y = params['origin'][1]

        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(5.0, self.publish_map)

        self.grid_msg = self._build_grid()

        self.get_logger().info("[occupancy_grid_publisher] Ready")

    def _build_grid(self):
        """
        Convert the input image into an OccupancyGrid message.

        Returns:
            OccupancyGrid: Populated occupancy grid message
        """
        img = Image.open(self.map_file).convert('L')    # grayscale
        arr = np.array(img)

        thresh = self.get_parameter('threshold').value

        # Convert to occupancy values
        grid = np.where(arr < thresh, 100, 0).astype(np.int8)

        # Flip vertically to match ROS coordinate convention
        grid = np.flipud(grid)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'

        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y

        msg.data = grid.flatten().tolist()

        self.get_logger().info(
            f'Loaded {self.map_file}: {msg.info.width}x{msg.info.height}, res={self.resolution}m'
        )

        return msg

    def publish_map(self):
        """
        Publish the occupancy grid at a fixed interval.
        """
        if self.grid_msg is None:
            return

        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid_msg)


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    rclpy.spin(OccupancyGridPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()