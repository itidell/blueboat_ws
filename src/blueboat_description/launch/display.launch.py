import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_path = get_package_share_directory('blueboat_description')
    xacro_file = PathJoinSubstitution([pkg_path, 'urdf', 'blueboat.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([pkg_path, 'rviz', 'urdf_config.rviz']) # We create this next

    robot_description_config = Command(['xacro ', xacro_file])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]
    )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_urdf',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        node_robot_state_publisher,
        node_rviz
    ])
