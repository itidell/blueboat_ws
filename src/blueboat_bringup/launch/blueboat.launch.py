# ~/blueboat_ws/src/blueboat_bringup/launch/blueboat.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition # Import IfCondition

# Opaque function allows late evaluation of launch configuration arguments
def launch_setup(context, *args, **kwargs):
    # --- Get Launch Argument Values ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')

    # --- Package Paths ---
    blueboat_description_pkg = get_package_share_directory('blueboat_description')
    blueboat_hardware_interface_pkg = get_package_share_directory('blueboat_hardware_interface')
    blueboat_sensors_pkg = get_package_share_directory('blueboat_sensors')
    blueboat_localization_pkg = get_package_share_directory('blueboat_localization')

    # --- File Paths ---
    xacro_file = PathJoinSubstitution([blueboat_description_pkg, 'urdf', 'blueboat.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([blueboat_description_pkg, 'rviz', 'urdf_config.rviz'])

    # --- Robot Description ---
    robot_description_config = Command(['xacro ', xacro_file])

    # --- Nodes and Included Launch Files ---

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # --- Corrected parameters list ---
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
        # --- End corrected parameters list ---
    )

    # 2. Hardware Interface Launch File
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_hardware_interface_pkg, 'launch', 'hardware_interface.launch.py'])
        )
    )

    # 3. Sensors Launch File
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_sensors_pkg, 'launch', 'sensors.launch.py'])
        )
    )

    # 4. Localization Launch File (EKF + NavSat)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_localization_pkg, 'launch', 'localization.launch.py'])
        )
    )

    # 5. RViz Node (Conditional)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_bringup',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    # --- Return list of actions to launch ---
    launch_actions = []
    launch_actions.append(node_robot_state_publisher)
    launch_actions.append(hardware_interface_launch)
    launch_actions.append(sensors_launch)
    launch_actions.append(localization_launch)
    launch_actions.append(node_rviz)

    return launch_actions


def generate_launch_description():
    # --- Declare Launch Arguments ---
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz with the bringup')
    )

    # --- Assemble Launch Description ---
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
