#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'selected_shape', 10)
        self.get_logger().info("ShapeNode started.")
        self.get_logger().info("Available commands: lemniscate, butterfly, heart, rose, stop, clear")

        # run input loop in a separate thread
        thread = threading.Thread(target=self.input_loop, daemon=True)
        thread.start()

    def input_loop(self):
        while rclpy.ok():
            shape = input("Enter shape (lemniscate/butterfly/heart/rose/stop/clear): ").strip()
            msg = String()
            msg.data = shape
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {shape}")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
