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
# This is good practice for accessing LaunchConfigurations within the launch description generation
def launch_setup(context, *args, **kwargs):
    # --- Get Launch Argument Values (evaluated at launch time) ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')

    # --- Package Paths ---
    # Find the share directories of the relevant packages needed in this scope
    blueboat_description_pkg = get_package_share_directory('blueboat_description')
    blueboat_hardware_interface_pkg = get_package_share_directory('blueboat_hardware_interface')
    blueboat_sensors_pkg = get_package_share_directory('blueboat_sensors')
    blueboat_localization_pkg = get_package_share_directory('blueboat_localization')
    blueboat_navigation_pkg = get_package_share_directory('blueboat_navigation') # Needed for Day 4

    # --- File Paths ---
    # Path to the URDF/Xacro file
    xacro_file = PathJoinSubstitution([blueboat_description_pkg, 'urdf', 'blueboat.urdf.xacro'])
    # Path to the RViz config file we want to use (the navigation one)
    rviz_config_file = PathJoinSubstitution([blueboat_description_pkg, 'rviz', 'nav_config.rviz']) # Use nav_config

    # --- Robot Description ---
    # Process the Xacro file to generate the URDF XML string
    robot_description_config = Command(['xacro ', xacro_file])

    # --- Launch Actions List ---
    # We will add all nodes and includes to this list

    launch_actions = []

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time # Pass argument value
        }]
    )
    launch_actions.append(rsp_node)

    # 2. Hardware Interface Launch File (Placeholder Controller)
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_hardware_interface_pkg, 'launch', 'hardware_interface.launch.py'])
        )
        # No launch arguments needed to pass down currently
    )
    launch_actions.append(hardware_interface_launch)

    # 3. Sensors Launch File (Placeholders)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_sensors_pkg, 'launch', 'sensors.launch.py'])
        )
        # No launch arguments needed to pass down currently
    )
    launch_actions.append(sensors_launch)

    # 4. Localization Launch File (EKF + NavSat)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_localization_pkg, 'launch', 'localization.launch.py'])
        )
        # Pass use_sim_time if the included launch needs it (currently it doesn't declare it)
        # launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    launch_actions.append(localization_launch)

    # 5. Navigation Launch File (Nav2 Stack) - Added for Day 4
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([blueboat_navigation_pkg, 'launch', 'navigation.launch.py'])
        ),
        # Pass arguments down to the included navigation launch file
        launch_arguments={
            'use_sim_time': use_sim_time,
            # We could override params_file or map from here if needed,
            # but defaults are correctly set in navigation.launch.py
        }.items()
    )
    launch_actions.append(navigation_launch)

    # 6. RViz Node (Conditional on 'start_rviz' argument)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_bringup', # Use a distinct name
        arguments=['-d', rviz_config_file], # Use the nav_config file path
        output='screen',
        condition=IfCondition(start_rviz) # Check the boolean value of the launch argument
    )
    launch_actions.append(rviz_node)

    # Return the list of actions for the OpaqueFunction to execute
    return launch_actions


# Main launch description generation function
def generate_launch_description():
    # --- Declare Launch Arguments available to the user ---
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
            default_value='true', # Start RViz by default for easy testing
            description='Whether to start RViz with the bringup')
    )

    # --- Assemble Launch Description ---
    # Create the LaunchDescription object and add declared arguments
    ld = LaunchDescription(declared_arguments)
    # Add the OpaqueFunction which will evaluate arguments and generate the node/include actions
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
