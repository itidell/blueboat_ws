# ~/blueboat_ws/src/blueboat_hardware_interface/launch/hardware_interface.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueboat_hardware_interface',
            executable='base_controller_node',
            name='base_controller_node', # Optional: Assign a specific node name
            output='screen',
            parameters=[
                # Optional: Override default parameters here if needed
                # {'odom_frame': 'odom_custom'}
            ]
        )
    ])
