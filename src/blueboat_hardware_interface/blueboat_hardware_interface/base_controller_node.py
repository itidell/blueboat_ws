import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5); cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5); cr = np.cos(roll * 0.5); sr = np.sin(roll * 0.5)
    q = [0] * 4
    q[0] = cr * cp * cy + sr * sp * sy; q[1] = sr * cp * cy - cr * sp * sy; q[2] = cr * sp * cy + sr * cp * sy; q[3] = cr * cp * sy - sr * sp * cy
    return q

class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        self.get_logger().info('Placeholder Base Controller Node Started')
        self.declare_parameter('odom_publish_rate', 50.0); self.declare_parameter('odom_frame', 'odom'); self.declare_parameter('base_frame', 'base_link')
        self.odom_publish_rate = self.get_parameter('odom_publish_rate').value; self.odom_frame = self.get_parameter('odom_frame').value; self.base_frame = self.get_parameter('base_frame').value
        self.x = 0.0; self.y = 0.0; self.theta = 0.0; self.last_time = self.get_clock().now(); self.current_linear_x = 0.0; self.current_angular_z = 0.0
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.odom_publish_rate, self.publish_odometry)
        self.get_logger().info(f"Publishing fake odometry to /odom_raw at {self.odom_publish_rate} Hz")

    def cmd_vel_callback(self, msg):
        self.current_linear_x = msg.linear.x; self.current_angular_z = msg.angular.z

    def publish_odometry(self):
        current_time = self.get_clock().now(); dt = (current_time - self.last_time).nanoseconds / 1e9
        delta_x = self.current_linear_x * math.cos(self.theta) * dt; delta_y = self.current_linear_x * math.sin(self.theta) * dt; delta_theta = self.current_angular_z * dt
        self.x += delta_x; self.y += delta_y; self.theta += delta_theta
        odom_msg = Odometry(); odom_msg.header.stamp = current_time.to_msg(); odom_msg.header.frame_id = self.odom_frame; odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x; odom_msg.pose.pose.position.y = self.y; odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.w = q[0]; odom_msg.pose.pose.orientation.x = q[1]; odom_msg.pose.pose.orientation.y = q[2]; odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.twist.twist.linear.x = self.current_linear_x; odom_msg.twist.twist.linear.y = 0.0; odom_msg.twist.twist.angular.z = self.current_angular_z
        self.odom_pub.publish(odom_msg)
        t = TransformStamped(); t.header.stamp = current_time.to_msg(); t.header.frame_id = self.odom_frame; t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x; t.transform.translation.y = self.y; t.transform.translation.z = 0.0
        t.transform.rotation.w = q[0]; t.transform.rotation.x = q[1]; t.transform.rotation.y = q[2]; t.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(t)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args); node = BaseControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
