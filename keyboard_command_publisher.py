#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_key

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

    print("Use arrow keys to control the servos , basically the rover. Press ESC to exit.")

    linear = 0.0
    angular = 0.0
    speed = 0.5  
    turn = 0.5   

    while True:
        events = get_key()
        for event in events:
            if event.ev_type == 'Key':
                if event.code == 'KEY_UP' and event.state == 1:  
                    linear = speed
                elif event.code == 'KEY_DOWN' and event.state == 1:
                    linear = -speed
                elif event.code == 'KEY_LEFT' and event.state == 1:
                    angular = turn
                elif event.code == 'KEY_RIGHT' and event.state == 1:
                    angular = -turn
                elif event.code == 'KEY_ESC':
                    return  

                
                if event.state == 0:  
                    linear = 0.0
                    angular = 0.0

                node.send_command(linear, angular)

if __name__ == '__main__':
    main()
