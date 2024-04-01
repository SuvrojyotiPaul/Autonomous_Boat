#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from math import radians, cos, sin, atan2, sqrt, degrees

def calculate_distance(lat1, lon1, lat2, lon2):
    
    R = 6371000  
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2 - lat1)
    delta_lambda = radians(lon2 - lon1)
    a = sin(delta_phi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance

class GPSBasedServoControlNode(Node):
    def __init__(self):
        super().__init__('gps_based_servo_control_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_data_callback,
            10)
        self.factory = PiGPIOFactory()
        self.servo_left = Servo(17, pin_factory=self.factory)
        self.servo_right = Servo(27, pin_factory=self.factory)
        
        self.waypoints = [
            (40.0000, -74.0000),
            (40.0005, -74.0000),
            (40.0005, -73.9995),
            (40.0000, -73.9995),
            (40.0000, -74.0000),
            (40.0005, -73.9995)
        ]
        self.current_waypoint_index = 0  

    def gps_data_callback(self, msg):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping.')
            self.stop_servos()  
            return
        current_gps_data = (msg.latitude, msg.longitude)
        current_waypoint = self.waypoints[self.current_waypoint_index]
       
        if calculate_distance(*current_gps_data, *current_waypoint) < 10:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached.')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached. Stopping.')
                self.stop_servos()  
                return
        
        bearing = self.calculate_bearing(current_gps_data, current_waypoint)
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}, bearing: {bearing} degrees')
        
        self.adjust_servos_based_on_bearing(bearing)

    def calculate_bearing(self, start, end):
        start_lat, start_lon, end_lat, end_lon = map(radians, [*start, *end])
        delta_lon = end_lon - start_lon
        x = sin(delta_lon) * cos(end_lat)
        y = cos(start_lat) * sin(end_lat) - sin(start_lat) * cos(end_lat) * cos(delta_lon)
        bearing = atan2(x, y)
        bearing_degrees = (degrees(bearing) + 360) % 360  
        return bearing_degrees

    def adjust_servos_based_on_bearing(self, bearing):
        straight_threshold = 20  
        reverse_threshold = 185 
        
        
        if bearing <= straight_threshold or bearing >= 360 - straight_threshold:
            self.move_forward()
        
        #elif abs(bearing - 180) < reverse_threshold:
           # self.move_reverse()
        
        elif 0 < bearing <= 180:
            if bearing <= 90:  
                self.turn_slightly_right()
                
            else:  
                self.turn_sharply_right()
      
        else:
            if 270 <= bearing <= 360:  
                self.turn_slightly_left()
            else: 
                self.turn_sharply_left()

    def move_forward(self):
        
        self.servo_left.value = 1.0
        self.servo_right.value = 1.0

    def move_reverse(self):
        
        self.servo_left.value = -1.0
        self.servo_right.value = -1.0

    def turn_slightly_right(self):
       
        self.servo_left.value = 1.0
        self.servo_right.value = 0.2

    def turn_sharply_right(self):
       
        self.servo_left.value = 1.0
        self.servo_right.value = 0.0

    def turn_slightly_left(self):
        
        self.servo_left.value = 0.2
        self.servo_right.value = 1.0

    def turn_sharply_left(self):
        
        self.servo_left.value = 0.0
        self.servo_right.value = 1.0

    def stop_servos(self):
        self.servo_left.value = 0.0
        self.servo_right.value = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = GPSBasedServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
