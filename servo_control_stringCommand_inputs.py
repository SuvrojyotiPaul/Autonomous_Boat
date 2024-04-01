#!/usr/bin/env python3

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            String,
            'servo_commands',
            self.command_callback,
            10)
        self.subscription  
        
        factory = PiGPIOFactory()
        
        self.servo1 = Servo(17, pin_factory=factory)
        self.servo2 = Servo(27, pin_factory=factory)

    def command_callback(self, msg):
        command = msg.data
        if command == "left":
            self.servo1.min()
            self.servo2.min()
        elif command == "right":
            self.servo1.max()
            self.servo2.max()
        elif command == "stop":
            self.servo1.value = 0
            self.servo2.value = 0

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
