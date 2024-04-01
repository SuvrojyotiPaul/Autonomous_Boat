#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()

        
        twist.linear.x = msg.axes[1] * 0.5  
        twist.angular.z = msg.axes[3] * 2.0  

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
