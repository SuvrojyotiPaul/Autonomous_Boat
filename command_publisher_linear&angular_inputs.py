#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Linear: {linear}, Angular: {angular}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    print("Enter linear and angular velocities: ")
    while rclpy.ok():
        try:
            linear, angular = map(float, input("Format <linear> <angular>: ").split())
            node.send_command(linear, angular)
        except ValueError:
            print("Invalid input format. Please enter two floating-point numbers separated by a space.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
