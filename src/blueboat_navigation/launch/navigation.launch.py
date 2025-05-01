# ~/blueboat_ws/src/blueboat_navigation/launch/navigation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # --- Define Paths ---
    # Get the path to this package's share directory
    blueboat_nav_dir = get_package_share_directory('blueboat_navigation')
    # Define the default path to the Nav2 parameters file within this package
    default_params_file = os.path.join(blueboat_nav_dir, 'config', 'nav2_params.yaml')
    # Define the default path to the map YAML file within this package
    default_map_yaml_file = os.path.join(blueboat_nav_dir, 'maps', 'dummy_map.yaml')

    # --- Declare Launch Arguments ---
    # These arguments allow customization when this launch file is included elsewhere
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters file to use for Nav2 nodes')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml_file,
        description='Full path to map yaml file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn nodes for this launch script if they crash')

    # --- Get Launch Configuration Values ---
    # Retrieve the values of the arguments declared above
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file_config = LaunchConfiguration('params_file')
    map_config = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

    # --- Configure Parameters with RewrittenYaml ---
    # This allows substituting launch configuration values into the YAML file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_config # Substitutes the map path into map_server params
    }

    configured_params = RewrittenYaml(
            source_file=params_file_config,
            root_key='', # Use root namespace (no nesting needed for these nodes)
            param_rewrites=param_substitutions,
            convert_types=True)

    # --- List of Nodes to be Managed by Lifecycle Manager ---
    # IMPORTANT: This list must match the 'node_names' parameter given to the lifecycle_manager node below
    #            AND the 'name' parameter given to each individual node definition.
    #            AMCL is omitted intentionally because robot_localization handles map->odom.
    lifecycle_nodes = [
                        'map_server',
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator'
                       ]

    # --- Define Individual Node Actions ---

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server', # Must match name in lifecycle_nodes list
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params], # Pass the processed YAML file
            arguments=['--ros-args', '--log-level', 'INFO'], # Optional: Adjust log level
        )

    controller_server_node = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server', # Must match name in lifecycle_nodes list
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            # remappings=[('/cmd_vel', '/cmd_vel_nav')] # Optional remapping if needed
        )

    planner_server_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server', # Must match name in lifecycle_nodes list
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
        )

    behavior_server_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server', # Must match name in lifecycle_nodes list
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
        )

    bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator', # Must match name in lifecycle_nodes list
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
        )

    # Lifecycle Manager Node (manages the nodes defined above)
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation', # Use a distinct name for this manager
            output='screen',
            parameters=[ # Pass parameters directly as a list of dictionaries
                       {'use_sim_time': use_sim_time},
                       {'autostart': autostart},
                       {'node_names': lifecycle_nodes} # Pass the list of node names to manage
                       ]
        )

    # --- Create the Launch Description ---
    ld = LaunchDescription()

    # Add the declared launch arguments to the description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the node actions to the description
    ld.add_action(map_server_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(lifecycle_manager_node)

    return ld # Return the assembled launch description
