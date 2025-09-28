#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute


def generate_shape_points(shape, steps=800):
    points = []
    scale = 3.0  # default scale

    for i in range(steps):
        t = i * 12 * math.pi / steps   

        if shape == "lemniscate":
            x = math.cos(t) / (1 + math.sin(t) ** 2)
            y = (math.sin(t) * math.cos(t)) / (1 + math.sin(t) ** 2)
            scale = 4.0

        elif shape == "butterfly":
            r = math.exp(math.sin(t)) - 2 * math.cos(4 * t) + math.sin((2 * t - math.pi) / 24) ** 5
            x = r * math.cos(t)
            y = r * math.sin(t)
            scale = 0.6 

        elif shape == "heart":
            x = 16 * math.sin(t) ** 3
            y = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
            scale = 0.2

        elif shape == "rose":
            k = 5   
            r = 3 * math.cos(k * t)
            x = r * math.cos(t)
            y = r * math.sin(t)
            scale = 1.5

        else:
            return []  

        # center
        x = 5.5 + scale * x
        y = 5.5 + scale * y

        # clamp to screen
        x = min(max(x, 0.5), 10.5)
        y = min(max(y, 0.5), 10.5)

        points.append((x, y))
    return points


class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscriber_shape = self.create_subscription(String, '/selected_shape', self.shape_callback, 10)

        # service clients
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/clear service not available, waiting...")

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/turtle1/teleport_absolute service not available, waiting...")

        self.pose = None
        self.shape_points = []
        self.index = 0
        self.active_shape = None

        self.timer = self.create_timer(0.05, self.follow)
        self.get_logger().info("TurtleFollower started. Waiting for shape...")

    def shape_callback(self, msg):
        shape = msg.data.strip().lower()
        if shape == "stop":
            self.shape_points = []
            self.get_logger().info("Stopping drawing.")
            return

        if shape == "clear":
            self.shape_points = []
            self.index = 0
            self.active_shape = None
            self.clear_and_reset()
            return

        # generate shape points
        self.shape_points = generate_shape_points(shape)
        self.index = 0
        self.active_shape = shape

        if self.shape_points:
            # teleport to the first point before drawing
            first_x, first_y = self.shape_points[0]
            teleport_req = TeleportAbsolute.Request()
            teleport_req.x = first_x
            teleport_req.y = first_y
            teleport_req.theta = 0.0
            self.teleport_client.call_async(teleport_req)

            self.get_logger().info(f"Teleported to start of shape: {shape}")

        self.get_logger().info(f"Now drawing shape: {shape}")

    def clear_and_reset(self):
        clear_req = Empty.Request()
        self.clear_client.call_async(clear_req)

        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = 5.5
        teleport_req.y = 5.5
        teleport_req.theta = 0.0
        self.teleport_client.call_async(teleport_req)

        stop_twist = Twist()
        self.publisher.publish(stop_twist)

        self.get_logger().info("Canvas cleared and turtle reset to center.")

    def pose_callback(self, msg):
        self.pose = msg

    def follow(self):
        if self.pose is None or not self.shape_points or self.index >= len(self.shape_points):
            return

        target_x, target_y = self.shape_points[self.index]

        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        twist = Twist()
        if distance > 0.05:
            twist.linear.x = 1.5 * distance
            twist.angular.z = 3.0 * angle_error
        else:
            self.index += 1

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
