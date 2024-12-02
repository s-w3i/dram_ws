from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Get the path to the `ign_sim` package share directory
    ign_sim_share = get_package_share_directory('ign_sim')

    # Default path to the YAML file for nav graph
    default_yaml_file = PathJoinSubstitution([ign_sim_share, 'maps', '0.yaml'])
    default_rviz_file = PathJoinSubstitution([ign_sim_share, 'config', 'def.rviz'])

    # Declare launch arguments
    args = [
        DeclareLaunchArgument('viz_config_file', default_value=default_rviz_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='L1'),
        DeclareLaunchArgument('headless', default_value='0'),
        DeclareLaunchArgument('nav_graph_file', default_value=default_yaml_file, description='Path to the navigation graph YAML file'),
    ]

    # RViz2 execution
    rviz_action = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration('viz_config_file')],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # Nodes to launch after a delay
    delayed_nodes = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            Node(
                package='dram_viz',  # Replace with your package name
                executable='nav_graph_publisher',  # The name of your executable/script
                name='nav_graph_publisher',
                output='screen',
                parameters=[{'nav_graph_file': LaunchConfiguration('nav_graph_file')}]
            ),

            Node(
                package='dram_viz',
                executable='nav_graph_visualizer',
                name='nav_graph_visualizer',
            ),

            Node(
                package='dram_viz',
                executable='robot_visualizer',
                name='robot_visualizer',
            ),

            Node(
                package='dram_viz',
                executable='path_viz',
                name='path_visualizer',
            ),

            Node(
                package='rmf_visualization_floorplans',
                executable='floorplan_visualizer_node',
                name='floorplan_visualizer',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'initial_map_name': LaunchConfiguration('map_name')}
                ]
            )
        ]
    )

    return LaunchDescription(args + [rviz_action, delayed_nodes])
