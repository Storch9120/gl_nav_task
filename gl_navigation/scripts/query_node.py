#!/usr/bin/env python3

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
    def __init__(self):
        super().__init__('query_node')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.semantic_map_file = get_package_share_directory('gl_navigation') + '/config/semantic_map.json'
        self.loadSemMap()

        self.query_srv = self.create_service(Query, 'send_query', self.queryCb)
        self.get_logger().info(f'[query_node] Query Node Ready')


    def loadSemMap(self):
        try:
            with open(self.semantic_map_file) as file:
                self.semantic_map = json.load(file)
        except Exception as e:
            self.get_logger().error(f"[query_node::loadSemMap] Exception : {e}")

    def sendGoal(self, pose):
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
        # compare the embedded query against each label in the map then extract the pose
        embedded_query = mock_embedding(query)
        best_label, best_score, best_pose = None, -1.0, None
        for label, data in self.semantic_map.items():
            score = cosine_similarity(embedded_query, data['embedding'])
            print(f'[query_node::processQuery] Comparing with label "{label}" got score {score:.4f}')
            if score > best_score:
                best_score = score
                best_label = label
                best_pose = data['pose']
            
        if best_score < 0.5:
            self.get_logger().warn(f'[query_node::processQuery] No good match found for query "{query}" (best was "{best_label}" with score {best_score:.4f})')
            return None, None
        return best_label, best_pose


    def queryCb(self, request, response):
        if request.query is not None:
            label, pose = self.processQuery(request.query)
            self.get_logger().info(f'[query_node::queryCb] Best match label "{label}" with pose {pose}')
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
    rclpy.init(args=args)
    rclpy.spin(QueryNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()