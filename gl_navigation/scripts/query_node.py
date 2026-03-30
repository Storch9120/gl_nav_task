#!/usr/bin/env python3

"""
ROS2 node that converts natural language queries into navigation goals.

Pipeline:
    query (text) -> embedding -> similarity search -> semantic label -> pose -> Nav2 goal

Uses a semantic map with precomputed embeddings.
"""

import json
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from gl_navigation.srv import Query
from fake_VLM import mock_embedding, cosine_similarity


class QueryNode(Node):
    """
    Semantic query-based navigation.

    Responsibilities:
        - Load semantic map
        - Accept text queries via service
        - Perform embedding similarity search
        - Send navigation goal to Nav2
    """

    def __init__(self):
        """Initialize node, load semantic map, and create interfaces."""
        super().__init__('query_node')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.semantic_map_file = get_package_share_directory('gl_navigation') + '/config/semantic_map.json'
        self.loadSemMap()

        self.query_srv = self.create_service(Query, 'send_query', self.queryCb)

        self.get_logger().info(f'[query_node] Query Node Ready')

    def loadSemMap(self):
        """
        Load semantic map from JSON file.

        The map format:
        {
            "label": {
                "pose": [x, y],
                "embedding": [...]
            }
        }
        """
        try:
            with open(self.semantic_map_file) as file:
                self.semantic_map = json.load(file)
        except Exception as e:
            self.get_logger().error(f"[query_node::loadSemMap] Exception : {e}")

    def sendGoal(self, pose):
        """
        Send navigation goal to Nav2.

        Args:
            pose (list[float]): [x, y] position in map frame
        """
        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = pose[0]
        goal.pose.pose.position.y = pose[1]
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)

        self.get_logger().info(f'[query_node::sendGoal] Goal sent')

    def processQuery(self, query):
        """
        Process a natural language query using embedding similarity.

        Args:
            query (str): Input user query

        Returns:
            tuple[str, list] or (None, None):
                Best matching label and pose if found, else None
        """
        embedded_query = mock_embedding(query)

        best_label, best_score, best_pose = None, -1.0, None

        for label, data in self.semantic_map.items():
            score = cosine_similarity(embedded_query, data['embedding'])

            #print(f'[query_node::processQuery] Comparing with label "{label}" got score {score:.4f}')

            if score > best_score:
                best_score = score
                best_label = label
                best_pose = data['pose']

        if best_score < 0.5:
            self.get_logger().warn(
                f'[query_node::processQuery] No good match found for query "{query}" '
                f'(best was "{best_label}" with score {best_score:.4f})'
            )
            return None, None

        return best_label, best_pose


    def queryCb(self, request, response):
        """
        Service callback to handle incoming queries.

        Args:
            request: Query request containing text query
            response: Response object

        Returns:
            response: Updated response with success status and message
        """
        if request.query is not None:
            label, pose = self.processQuery(request.query)

            self.get_logger().info(
                f'[query_node::queryCb] Best match label "{label}" with pose {pose}'
            )

            if not pose:
                response.success = False
                response.message = "Failed to find label"
                return response
            else:
                self.sendGoal(pose)
                response.success = True
                response.message = f"Derived pose at x,y : ({pose[0]}, {pose[1]})"
                return response
        else:
            response.success = False
            response.message = "No query given"
            return response


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    rclpy.spin(QueryNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()