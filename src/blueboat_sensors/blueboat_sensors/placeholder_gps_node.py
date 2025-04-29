# ~/blueboat_ws/src/blueboat_sensors/blueboat_sensors/placeholder_gps_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np # Can use for adding noise later if desired

class PlaceholderGpsNode(Node):
    def __init__(self):
        super().__init__('placeholder_gps_node')
        self.declare_parameter('gps_frame_id', 'gps_link'); self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('initial_latitude', 40.7128); self.declare_parameter('initial_longitude', -74.0060) # Example: NYC
        self.declare_parameter('initial_altitude', 10.0); self.declare_parameter('horizontal_accuracy', 5.0); self.declare_parameter('vertical_accuracy', 10.0)
        self.gps_frame_id = self.get_parameter('gps_frame_id').value; publish_rate = self.get_parameter('publish_rate').value
        timer_period = 1.0 / publish_rate; self.latitude = self.get_parameter('initial_latitude').value
        self.longitude = self.get_parameter('initial_longitude').value; self.altitude = self.get_parameter('initial_altitude').value
        self.horizontal_variance = self.get_parameter('horizontal_accuracy').value ** 2; self.vertical_variance = self.get_parameter('vertical_accuracy').value ** 2
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Placeholder GPS: Pub '/gps/fix' frame '{self.gps_frame_id}' @{publish_rate} Hz")

    def timer_callback(self):
        msg = NavSatFix(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.gps_frame_id
        msg.status.status = NavSatStatus.STATUS_FIX; msg.status.service = NavSatStatus.SERVICE_GPS
        # Simulate slight random walk drift (optional)
        # self.latitude += np.random.normal(0, 0.000001)
        # self.longitude += np.random.normal(0, 0.000001)
        msg.latitude = self.latitude; msg.longitude = self.longitude; msg.altitude = self.altitude
        # Covariance (diagonal matrix)
        msg.position_covariance[0] = self.horizontal_variance; msg.position_covariance[4] = self.horizontal_variance; msg.position_covariance[8] = self.vertical_variance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args); node = PlaceholderGpsNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
