import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv

class GPSDataPublisher(Node):
    def __init__(self):
        super().__init__('gps_data_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)
        self.timer_period = 1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.csv_file_path = '/home/paul/ros2_ws/install/my_robot/share/my_robot/mock_gps_data.csv'
        self.csv_data = self.read_csv_data()

    def read_csv_data(self):
        data = []
        with open(self.csv_file_path, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                data.append(row)
        return data

    def timer_callback(self):
        if self.csv_data:
            gps_point = self.csv_data.pop(0)  
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.latitude = float(gps_point['latitude'])
            msg.longitude = float(gps_point['longitude'])
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing GPS Data: {msg.latitude}, {msg.longitude}")
        else:
            self.get_logger().info("End of GPS data.")

def main(args=None):
    rclpy.init(args=args)
    gps_data_publisher = GPSDataPublisher()
    rclpy.spin(gps_data_publisher)
    gps_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

