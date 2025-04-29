# ~/blueboat_ws/src/blueboat_sensors/launch/sensors.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This launch file simply starts all placeholder sensor nodes together
    return LaunchDescription([
        Node(
            package='blueboat_sensors',
            executable='placeholder_imu_node',
            name='placeholder_imu_node', # Explicit node name
            output='screen'
            # Add parameters here if you want to override defaults from the launch file
            # parameters=[{'publish_rate': 100.0}]
        ),
        Node(
            package='blueboat_sensors',
            executable='placeholder_gps_node',
            name='placeholder_gps_node', # Explicit node name
            output='screen'
            # parameters=[{'publish_rate': 5.0}]
        ),
        Node(
            package='blueboat_sensors',
            executable='placeholder_lidar_node',
            name='placeholder_lidar_node', # Explicit node name
            output='screen'
            # parameters=[{'publish_rate': 15.0}]
        ),
    ])
