#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
from nav2_simple_commander.utils import euler_to_quaternion
from nav2_msgs.srv import DynamicEdges
import enum
import time
import os
from geojson_reader import Graph
from ament_index_python.packages import get_package_share_directory
import sign_detector
from sign_detector import DetectedSign

class RouteExampleNode(Node):
    def __init__(self):
        super().__init__('route_example')

        graph_path = os.path.join(get_package_share_directory('ramson_bot'), 'graphs', 'my_graph.geojson')
        self.graph = Graph(graph_path)

        self.trigger_nodes = self.graph.need_detection_nodes

        # Connect to DynamicEdges service
        self.client = self.create_client(DynamicEdges, 'route_server/DynamicEdgesScorer/adjust_edges')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for adjust_edges service...')
        self.get_logger().info('Connected to adjust_edges service')

        # Initialize navigator and wait until Nav2 is active
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

    def adjust_closed_edges(self, closed_edges: list[int]) -> None:
        req = DynamicEdges.Request()
        req.closed_edges = closed_edges
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Edges {closed_edges} updated successfully')
        else:
            self.get_logger().error(f'Failed to update edges {closed_edges}')

    def do_sign_detection(self) -> DetectedSign:
        # time.sleep(2)
        return sign_detector.detect_sign()

    def get_edges_to_close(self, node_id: int, sign: DetectedSign) -> list[int]:
        edge_ids = []
        for direction in sign.closed_directions:
            edge = self.graph.get_turn_edge(node_id, direction)
            if edge:
                edge_ids.append(edge.id)
        return edge_ids
    
    def on_node_change(self, new_node_id, route_tracking_task, goal_id):
        if new_node_id in self.trigger_nodes:
            self.get_logger().info(f"Passed trigger node {new_node_id}")
            self.navigator.cancelTask()
            detected_sign = self.do_sign_detection()
            if detected_sign != DetectedSign.NONE:
                self.get_logger().info(f"Detected sign: {detected_sign.name}")
                edges_to_close = self.get_edges_to_close(new_node_id, detected_sign)
                self.get_logger().info(f"Edges to close: {edges_to_close}")
                if len(edges_to_close):
                    self.adjust_closed_edges(edges_to_close)
            else:
                self.get_logger().info(f"No sign detected")
            route_tracking_task = self.navigator.getAndTrackRoute(new_node_id, goal_id)
        return route_tracking_task

    def navigate_route(self, start_id: int, goal_id: int) -> None:
        route_tracking_task = self.navigator.getAndTrackRoute(start_id, goal_id)
        task_canceled = False
        last_feedback = None
        follow_path_task = RunningTask.NONE

        while not self.navigator.isTaskComplete(task=route_tracking_task):
            feedback = self.navigator.getFeedback(task=route_tracking_task)
            while feedback is not None:
                if last_feedback != None and feedback.last_node_id != 0 and last_feedback.last_node_id != 0 \
                    and feedback.last_node_id != last_feedback.last_node_id:
                    print(f"this: {feedback.last_node_id} -> {feedback.next_node_id}")
                    if last_feedback:
                        print(f"last: {last_feedback.last_node_id} -> {last_feedback.next_node_id}")
                    self.get_logger().info(
                        f"Passed node {feedback.last_node_id} to {feedback.next_node_id} "
                        f"on edge {feedback.current_edge_id}")
                    # route_tracking_task = self.on_node_change(feedback.last_node_id, route_tracking_task, goal_id)
                    if feedback.last_node_id in self.trigger_nodes:
                        self.get_logger().info(f"Passed trigger node {feedback.last_node_id}")
                        self.navigator.cancelTask()
                        detected_sign = self.do_sign_detection()
                        if detected_sign != DetectedSign.NONE:
                            self.get_logger().info(f"Detected sign: {detected_sign.name}")
                            edges_to_close = self.get_edges_to_close(feedback.last_node_id, detected_sign)
                            self.get_logger().info(f"Edges to close: {edges_to_close}")
                            if len(edges_to_close):
                                self.adjust_closed_edges(edges_to_close)
                        else:
                            self.get_logger().info(f"No sign detected")
                        route_tracking_task = self.navigator.getAndTrackRoute(feedback.last_node_id, goal_id)
                        feedback = self.navigator.getFeedback(task=route_tracking_task)
                        feedback.rerouted = True
                if feedback.rerouted:  # or follow_path_task == RunningTask.None
                    self.get_logger().info('Rerouted, following new path')
                    follow_path_task = self.navigator.followPath(feedback.path)
                    # May instead use the waypoint follower
                    # (or nav through poses) and use the route's sparse nodes!
                    # print("Passing route to waypoint follower!")
                    # nodes = [toPoseStamped(x.position, feedback.route.header) for x in feedback.route.nodes]
                    # navigator.followWaypoints(nodes)
                    # Or navigator.navigateThroughPoses(nodes)
                    # Consider sending only the first few and iterating
                last_feedback = feedback
                feedback = self.navigator.getFeedback(task=route_tracking_task)
            if self.navigator.isTaskComplete(task=follow_path_task):
                self.get_logger().info('Controller or waypoint follower server completed its task!')
                self.navigator.cancelTask()
                task_canceled = True

        while not self.navigator.isTaskComplete(task=follow_path_task) and not task_canceled:
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled!')
        else:
            self.get_logger().error(f'Goal failed with code: {result}')
    
    def rotate(self, angle):
        task_id = self.navigator.spin(spin_dist=angle, time_allowance=10)

        while not self.navigator.isTaskComplete(task=task_id):
            # feedback = self.navigator.getFeedback(task=task_id)
            pass
    
    def move_forward(self, dist, speed, time_allowance=5):
        task_id = self.navigator.driveOnHeading(dist=dist, speed=speed, time_allowance=time_allowance)
        while not self.navigator.isTaskComplete(task=task_id):
            # feedback = self.navigator.getFeedback(task=task_id)
            pass
    
    def move_back(self, dist, speed, time_allowance=5):
        task_id = self.navigator.backup(backup_dist=dist, backup_speed=speed, time_allowance=time_allowance)
        while not self.navigator.isTaskComplete(task=task_id):
            # feedback = self.navigator.getFeedback(task=task_id)
            pass
    
    def set_initial_pose(self, start_x, start_y, start_yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = start_x
        initial_pose.pose.position.y = start_y
        initial_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, start_yaw)
        self.navigator.setInitialPose(initial_pose)


def main(args=None):
    rclpy.init(args=args)
    node = RouteExampleNode()

    # Set initial pose
    start_x = 0.0
    start_y = 0.0
    start_yaw = 0.0
    node.set_initial_pose(start_x, start_y, start_yaw)

    # Calibrate position
    # node.move_forward(dist=0.2, speed=0.15)
    node.move_back(dist=0.7, speed=0.15)
    # node.rotate(math.pi/2)
    # node.rotate(-math.pi/2)

    # Navigate
    start_id = 1
    goal_id = 10

    node.navigate_route(13, 10)
    node.navigate_route(10, 1)

    # node.navigate_route(start_id, goal_id)
    # node.navigate_route(goal_id, start_id)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
