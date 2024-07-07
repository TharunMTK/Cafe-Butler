# Cafe-Butler
# Step 1
Create a new package for your robot butler project. This package will contain all the nodes, launch files, and configuration files needed.

![image](https://github.com/TharunMTK/Cafe-Butler/assets/112540520/61cff92c-d69b-4734-81a5-2851e3edf79c)

# Step 2

 Define the Robot's Behavior,Create a Python node to handle the robot's behavior based on the scenarios provided. This node will manage the robot's tasks, handle timeouts, and confirmations.

robot_butler.py
Create a new Python script in your package's src directory:

##mkdir -p ~/robot_butler_ws/src/robot_butler/robot_butler
touch ~/robot_butler_ws/src/robot_butler/robot_butler/robot_butler.py
chmod +x ~/robot_butler_ws/src/robot_butler/robot_butler/robot_butler.py

# Step 3 

Add the code to 'robot_butler.py'


    #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

##class RobotButler(Node):
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

# Step 4 Navigation

You will need to integrate a navigation stack to allow the robot to move between the home position, kitchen, and tables. This can be done using the nav2 stack in ROS2.

sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup


# Step 5 Create Launch Files

Create a launch file to start all the necessary nodes.

robot_butler_launch.py
Create a launch file in the launch directory of your package:

mkdir -p ~/robot_butler_ws/src/robot_butler/launch
touch ~/robot_butler_ws/src/robot_butler/launch/robot_butler_launch.py

And add the following code to robot_butler_launch.py:

from launch import LaunchDescription
from launch_ros.actions import Node

##def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_butler',
            executable='robot_butler',
            name='robot_butler',
            output='screen'
        ),
        # Add other nodes such as navigation2 bringup here
    ])


# Step 6. Build and Run the Package  

cd ~/robot_butler_ws
colcon build
source install/setup.bash

And run the launch file:

##ros2 launch robot_butler robot_butler_launch.py  



![image](https://github.com/TharunMTK/Cafe-Butler/assets/112540520/f91bf0e3-2a0d-48da-92df-76869b94815b)

