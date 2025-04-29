# ~/blueboat_ws/src/blueboat_localization/launch/localization.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package share directory
    localization_pkg_dir = get_package_share_directory('blueboat_localization')

    # Paths to configuration files
    ekf_config_path = os.path.join(localization_pkg_dir, 'config', 'ekf.yaml')
    navsat_config_path = os.path.join(localization_pkg_dir, 'config', 'navsat_transform.yaml')

    # Declare launch arguments
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false') # Get from parent launch file

    # Start EKF node (odom instance)
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom', # Must match the name in ekf.yaml
        output='screen',
        parameters=[ekf_config_path],
        # If using simulation time, uncomment this:
        # parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
    )

    # Start Navsat transform node
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node', # Must match the name in navsat_transform.yaml
        output='screen',
        parameters=[navsat_config_path],
        # If using simulation time, uncomment this:
        # parameters=[navsat_config_path, {'use_sim_time': use_sim_time}],
        remappings=[ # Remap default topics to match our setup
                    ('imu', '/imu/data_raw'), # Takes sensor_msgs/Imu
                    ('gps/fix', '/gps/fix'), # Takes sensor_msgs/NavSatFix
                    ('odometry/filtered', '/odometry/filtered'), # Takes nav_msgs/Odometry from EKF
                    # --- Outputs ---
                    ('odometry/gps', '/odometry/gps'), # Publishes nav_msgs/Odometry
                    ('gps/filtered', '/gps/filtered') # Publishes sensor_msgs/NavSatFix
                   ]
    )

    return LaunchDescription([
        ekf_odom_node,
        navsat_node
    ])
