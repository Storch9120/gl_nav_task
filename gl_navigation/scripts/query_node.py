#!/usr/bin/env python3

import json
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from gl_navigation.srv import Query


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
        # search the query label in the map then extract 
        return self.semantic_map[query]["pose"] if query in self.semantic_map else None


    def queryCb(self, request, response):
        print(request.query)
        if request.query is not None:
            pose = self.processQuery(request.query)
            print(pose)
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