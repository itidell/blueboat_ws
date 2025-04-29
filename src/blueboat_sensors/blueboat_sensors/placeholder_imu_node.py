# ~/blueboat_ws/src/blueboat_sensors/blueboat_sensors/placeholder_imu_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np # For quaternion and potential noise generation

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w, x, y, z)
    """
    cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5); cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5); cr = np.cos(roll * 0.5); sr = np.sin(roll * 0.5)
    q = [0] * 4
    q[0] = cr * cp * cy + sr * sp * sy  # w
    q[1] = sr * cp * cy - cr * sp * sy  # x
    q[2] = cr * sp * cy + sr * cp * sy  # y
    q[3] = cr * cp * sy - sr * sp * cy  # z
    return q # <--- Ensure NO text follows this line within this function

class PlaceholderImuNode(Node): # <--- This line must NOT be indented
    def __init__(self):
        # --- Start of indented block (4 spaces typical) ---
        super().__init__('placeholder_imu_node')
        self.declare_parameter('imu_frame_id', 'imu_link'); self.declare_parameter('publish_rate', 50.0)
        self.imu_frame_id = self.get_parameter('imu_frame_id').value; publish_rate = self.get_parameter('publish_rate').value
        timer_period = 1.0 / publish_rate; self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10) # Standard topic name
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Placeholder IMU: Pub '/imu/data_raw' frame '{self.imu_frame_id}' @{publish_rate} Hz")
        # --- End of indented block ---

    def timer_callback(self):
        # --- Start of indented block ---
        msg = Imu(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.imu_frame_id
        roll, pitch, yaw = 0.0, 0.0, 0.0; q = quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.w = q[0]; msg.orientation.x = q[1]; msg.orientation.y = q[2]; msg.orientation.z = q[3]
        msg.angular_velocity.x = 0.0; msg.angular_velocity.y = 0.0; msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0; msg.linear_acceleration.y = 0.0; msg.linear_acceleration.z = 0.0 # Set to 9.81 if simulating gravity effect
        # Covariances (Important for sensor fusion!)
        msg.orientation_covariance[0] = 0.01; msg.orientation_covariance[4] = 0.01; msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.05; msg.angular_velocity_covariance[4] = 0.05; msg.angular_velocity_covariance[8] = 0.05
        msg.linear_acceleration_covariance[0] = 0.1; msg.linear_acceleration_covariance[4] = 0.1; msg.linear_acceleration_covariance[8] = 0.1
        self.publisher_.publish(msg)
        # --- End of indented block ---

def main(args=None): # <--- This line must NOT be indented
    rclpy.init(args=args); node = PlaceholderImuNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': # <--- This line must NOT be indented
    main()
