# ~/blueboat_ws/src/blueboat_sensors/blueboat_sensors/placeholder_lidar_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan # <--- Needs LaserScan
import math
import numpy as np

class PlaceholderLidarNode(Node): # <--- Correct class name
    def __init__(self):
        super().__init__('placeholder_lidar_node') # <--- Correct node name
        # --- Parameters specific to Lidar ---
        self.declare_parameter('lidar_frame_id', 'lidar_link'); self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('angle_min', -math.pi); self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment_deg', 1.0); self.declare_parameter('range_min', 0.15); self.declare_parameter('range_max', 8.0)
        self.declare_parameter('default_range', 5.0); self.declare_parameter('obstacle_range', 1.5)
        self.declare_parameter('obstacle_angle_deg', 0.0); self.declare_parameter('obstacle_width_deg', 10.0) # Width of obstacle in degrees
        # --- Get parameters ---
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value; publish_rate = self.get_parameter('publish_rate').value
        timer_period = 1.0 / publish_rate; self.angle_min_rad = self.get_parameter('angle_min').value
        self.angle_max_rad = self.get_parameter('angle_max').value; angle_increment_deg = self.get_parameter('angle_increment_deg').value
        self.angle_increment_rad = math.radians(angle_increment_deg); self.range_min_m = self.get_parameter('range_min').value
        self.range_max_m = self.get_parameter('range_max').value; self.default_range_m = self.get_parameter('default_range').value
        self.obstacle_range_m = self.get_parameter('obstacle_range').value; obstacle_angle_rad = math.radians(self.get_parameter('obstacle_angle_deg').value)
        obstacle_half_width_rad = math.radians(self.get_parameter('obstacle_width_deg').value / 2.0)
        self.num_readings = int(round((self.angle_max_rad - self.angle_min_rad) / self.angle_increment_rad)) + 1
        self.obstacle_start_angle = obstacle_angle_rad - obstacle_half_width_rad; self.obstacle_end_angle = obstacle_angle_rad + obstacle_half_width_rad
        # --- Publisher ---
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10) # <--- Publishes LaserScan
        # --- Timer ---
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Placeholder LiDAR: Pub {self.num_readings} readings to /scan frame '{self.lidar_frame_id}' @{publish_rate} Hz") # <--- Correct log message

    def timer_callback(self):
        msg = LaserScan(); current_time = self.get_clock().now().to_msg(); msg.header.stamp = current_time; msg.header.frame_id = self.lidar_frame_id
        msg.angle_min = self.angle_min_rad; msg.angle_max = self.angle_max_rad; msg.angle_increment = self.angle_increment_rad
        msg.time_increment = 0.0; msg.scan_time = (1.0 / self.get_parameter('publish_rate').value) * 0.9 # Simulate scan time slightly less than publish interval
        msg.range_min = self.range_min_m; msg.range_max = self.range_max_m
        ranges = np.full(self.num_readings, self.default_range_m) # Initialize with default range
        # Add simulated obstacle
        for i in range(self.num_readings):
            current_angle = self.angle_min_rad + i * self.angle_increment_rad
            if self.obstacle_start_angle <= current_angle <= self.obstacle_end_angle:
                # Add potential noise: + np.random.normal(0, 0.05)
                obstacle_reading = self.obstacle_range_m
                # Clamp reading to valid range
                ranges[i] = max(self.range_min_m, min(self.range_max_m, obstacle_reading))

            # Handle out-of-range values explicitly (optional: can also leave as default_range if it's within max)
            if ranges[i] < self.range_min_m or ranges[i] > self.range_max_m:
                 ranges[i] = float('inf') # Use infinity for out of range readings

        msg.ranges = ranges.tolist() # Convert numpy array to list
        # msg.intensities = [] # Leave intensities empty
        self.publisher_.publish(msg)

def main(args=None): # <--- Correct main function
    rclpy.init(args=args); node = PlaceholderLidarNode() # <--- Instantiates correct node
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': # <--- Correct check
    main()
