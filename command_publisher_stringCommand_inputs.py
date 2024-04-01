#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'servo_commands', 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    print("Enter commands (left, right, stop) to control the servos:")
    while rclpy.ok():
        command = input("Command: ").strip()
        if command.lower() in ["left", "right", "stop"]:
            node.send_command(command.lower())
        else:
            print("Invalid command. Valid commands are: left, right, stop")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
