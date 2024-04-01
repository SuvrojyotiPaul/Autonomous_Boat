#!/usr/bin/env python3

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.command_callback,
            10)
        self.subscription  
        factory = PiGPIOFactory()
        self.servo1 = Servo(17, pin_factory=factory)
        self.servo2 = Servo(27, pin_factory=factory)

    def command_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        
        left_speed = linear_vel - angular_vel
        right_speed = linear_vel + angular_vel

        
        self.servo1.value = self.convert_speed_to_servo_value(left_speed)
        self.servo2.value = self.convert_speed_to_servo_value(right_speed)

    def convert_speed_to_servo_value(self, speed):
        
        return max(min(speed, 1), -1)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
