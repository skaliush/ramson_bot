#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
from nav2_simple_commander.utils import euler_to_quaternion
from nav2_msgs.srv import DynamicEdges

class RouteExampleNode(Node):
    def __init__(self):
        super().__init__('route_example')

        # Declare parameters
        self.declare_parameter('start_pose.x', 0.0)
        self.declare_parameter('start_pose.y', 0.0)
        self.declare_parameter('start_pose.yaw', 0.0)
        self.declare_parameter('goal_pose.x', 0.0)
        self.declare_parameter('goal_pose.y', 0.0)
        self.declare_parameter('goal_pose.yaw', 0.0)
        self.declare_parameter('start_id', 1)
        self.declare_parameter('goal_id', 10)

        # Retrieve parameters
        self.start_x = self.get_parameter('start_pose.x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_pose.y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_pose.yaw').get_parameter_value().double_value
        self.goal_x = self.get_parameter('goal_pose.x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_pose.y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_pose.yaw').get_parameter_value().double_value
        self.start_id = self.get_parameter('start_id').get_parameter_value().integer_value
        self.goal_id = self.get_parameter('goal_id').get_parameter_value().integer_value

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
        """Call DynamicEdges service to adjust the given closed edges."""
        req = DynamicEdges.Request()
        req.closed_edges = closed_edges
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Edges {closed_edges} updated successfully')
        else:
            self.get_logger().error(f'Failed to update edges {closed_edges}')
    
    def on_node_change(self, feedback):
        # print(feedback)
        trigger_nodes = [1]
        self.get_logger().info(
            f"Passed node {feedback.last_node_id} to {feedback.next_node_id} "
            f"on edge {feedback.current_edge_id}")
        if feedback.last_node_id in trigger_nodes:
            node.

    def navigate_route(self, start_id: int, goal_id: int) -> None:
        """Plan and follow a route between two node IDs."""
        route_task = self.navigator.getAndTrackRoute(start_id, goal_id)
        task_canceled = False
        last_feedback = None
        follow_path_task = RunningTask.NONE

        while not self.navigator.isTaskComplete(task=route_task):
            feedback = self.navigator.getFeedback(task=route_task)
            while feedback:
                if (not last_feedback or
                    feedback.last_node_id != last_feedback.last_node_id or
                    feedback.next_node_id != last_feedback.next_node_id):
                    self.on_node_change(feedback)
                    
                last_feedback = feedback
                if feedback.rerouted:
                    self.get_logger().info('Rerouted, following new path')
                    follow_path_task = self.navigator.followPath(feedback.path)
                feedback = self.navigator.getFeedback(task=route_task)

            # Check if followPath task completed
            if self.navigator.isTaskComplete(task=follow_path_task):
                self.get_logger().info('Controller/WPF task completed, canceling route')
                self.navigator.cancelTask()
                task_canceled = True

        # Wait for followPath to finish if not canceled
        while not self.navigator.isTaskComplete(task=follow_path_task) and not task_canceled:
            pass

        # Check result
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


def main(args=None):
    rclpy.init(args=args)
    node = RouteExampleNode()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.pose.position.x = node.start_x
    initial_pose.pose.position.y = node.start_y
    initial_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, node.start_yaw)
    node.navigator.setInitialPose(initial_pose)

    # Calibrate position
    node.move_forward(dist=0.2, speed=0.15)
    node.move_back(dist=0.2, speed=0.15)
    # node.rotate(math.pi/2)
    # node.rotate(-math.pi/2)

    # Navigate
    node.navigate_route(node.start_id, node.goal_id)
    node.navigate_route(node.goal_id, node.start_id)

    node.adjust_closed_edges([16, 22])

    # node.navigate_route(node.start_id, node.goal_id)
    # node.navigate_route(node.goal_id, node.start_id)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
