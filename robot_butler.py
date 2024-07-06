#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotButler(Node):
    def __init__(self):
        super().__init__('robot_butler')
        self.order_subscriber = self.create_subscription(String, 'order_topic', self.order_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_task = None
        self.task_queue = []

    def order_callback(self, msg):
        order = msg.data
        self.get_logger().info(f'Received order: {order}')
        self.task_queue.append(order)
        self.process_tasks()

    def timer_callback(self):
        # Implement the timeout and task confirmation logic here
        pass

    def process_tasks(self):
        if self.current_task is None and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            self.get_logger().info(f'Processing task: {self.current_task}')
            # Add logic to move the robot to the kitchen and then to the table

    def move_to(self, location):
        self.get_logger().info(f'Moving to {location}')
        # Add logic to move the robot to the specified location
        # Example: self.navigation_client.send_goal(location)

    def handle_task_completion(self):
        self.get_logger().info(f'Task {self.current_task} completed')
        self.current_task = None
        self.process_tasks()

def main(args=None):
    rclpy.init(args=args)
    robot_butler = RobotButler()
    rclpy.spin(robot_butler)
    robot_butler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
