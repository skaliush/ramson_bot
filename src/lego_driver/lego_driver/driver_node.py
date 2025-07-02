#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import time
import threading
import socket
import struct
import tf_transformations

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.ip = '192.168.0.1'
        self.output_signals = [0.0, 0.0]
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            TwistStamped, '/cmd_vel', self.listener_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.stop_threads = threading.Event()
        # Start connection management thread
        self.manage_thread = threading.Thread(target=self.manage_connection, daemon=True)
        self.manage_thread.start()

    def listener_callback(self, msg: TwistStamped):
        # Update desired velocities
        self.output_signals[0] = msg.twist.linear.x
        self.output_signals[1] = msg.twist.angular.z

    def manage_connection(self):
        while rclpy.ok():
            try:
                self.get_logger().info('Waiting for connection to EV3...')
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.ip, 8089))
                s.settimeout(None)
                self.get_logger().info('Connected to EV3')

                self.stop_threads.clear()
                # Start send and receive threads
                send_thread = threading.Thread(target=self.send_data, args=(s,), daemon=True)
                recv_thread = threading.Thread(target=self.receive_data, args=(s,), daemon=True)
                send_thread.start()
                recv_thread.start()

                # Monitor threads until a disconnection occurs
                while rclpy.ok() and not self.stop_threads.is_set():
                    time.sleep(0.1)

                # Clean up socket on disconnect
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                s.close()
                self.get_logger().warn('Connection lost, retrying in 5 seconds...')
                time.sleep(5)
            except Exception as e:
                self.get_logger().error(f'Connection error: {e}')
                time.sleep(5)

    def send_data(self, s: socket.socket):
        T = 1.0 / 20.0
        while rclpy.ok() and not self.stop_threads.is_set():
            start = time.time()
            try:
                packed = struct.pack('>ff', self.output_signals[0], self.output_signals[1])
                sent = s.send(packed)
                if sent == 0:
                    raise RuntimeError('Socket connection broken')
            except Exception as e:
                self.get_logger().error(f'Send error: {e}')
                self.stop_threads.set()
                break
            elapsed = time.time() - start
            time.sleep(max(0, T - elapsed))

    def receive_data(self, s: socket.socket):
        T = 1.0 / 100.0
        while rclpy.ok() and not self.stop_threads.is_set():
            start = time.time()
            try:
                chunk = s.recv(20)
                if not chunk:
                    raise RuntimeError('Socket connection broken')
                x, y, th, v, w = struct.unpack('>fffff', chunk)
                self.publish_odometry(x, y, th, v, w)
            except Exception as e:
                self.get_logger().error(f'Receive error: {e}')
                self.stop_threads.set()
                break
            elapsed = time.time() - start
            time.sleep(max(0, T - elapsed))

    def publish_odometry(self, x, y, th, v, w):
        msg = Odometry()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0, 0, th)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.publisher.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_threads.set()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
