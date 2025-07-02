#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges

class EdgeModifier(Node):

    def __init__(self):
        super().__init__('edge_modifier_client')
        self.client = self.create_client(DynamicEdges, 'route_server/DynamicEdgesScorer/adjust_edges')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ждем сервис adjust_edges...')

    # nav2_msgs/srv/DynamicEdges:
    # uint16[] closed_edges
    # uint16[] opened_edges
    # EdgeCost[] adjust_edges
    #     uint16 edgeid
    #     float32 cost    
    def send_request(self):
        req = DynamicEdges.Request()
        req.closed_edges = [16, 22]
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Рёбра обновлены успешно')
        else:
            self.get_logger().error('Не удалось обновить рёбра')

def main(args=None):
    rclpy.init(args=args)
    client = EdgeModifier()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
