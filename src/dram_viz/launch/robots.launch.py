from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare Launch Arguments for Parameters
    sliding_window_size_arg = DeclareLaunchArgument(
        'sliding_window_size',
        default_value='5',
        description='Sliding window size for resource allocator.'
    )

    navigate_url_arg = DeclareLaunchArgument(
        'navigate_url',
        default_value='http://127.0.0.1:22011/open-rmf/rmf_demos_fm/navigate/',
        description='URL for FastAPI navigate endpoint.'
    )

    status_url_arg = DeclareLaunchArgument(
        'status_url',
        default_value='http://127.0.0.1:22011/open-rmf/rmf_demos_fm/status/',
        description='URL for FastAPI status endpoint.'
    )

    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.5',
        description='Distance threshold in meters to consider waypoint as passed.'
    )

    # Launch Configurations to use the declared arguments
    sliding_window_size = LaunchConfiguration('sliding_window_size')
    navigate_url = LaunchConfiguration('navigate_url')
    status_url = LaunchConfiguration('status_url')
    distance_threshold = LaunchConfiguration('distance_threshold')

    # Define Nodes
    resource_manager_node = Node(
        package='dram_plan',
        executable='resource_manager',
        name='resource_manager',
        output='screen',
    )

    path_planner_node = Node(
        package='dram_plan',
        executable='path_planner',
        name='path_planner',
        output='screen',
        emulate_tty=True  # Ensures that output is visible in screen
    )

    resource_allocator_node = Node(
        package='dram_plan',
        executable='resource_allocator',
        name='resource_allocator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'sliding_window_size': sliding_window_size
        }]
    )

    command_to_fastapi_service_node = Node(
        package='dram_plan',
        executable='command_to_fastapi_service',
        name='command_to_fastapi_service',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'navigate_url': navigate_url,
            'status_url': status_url,
            'distance_threshold': distance_threshold
        }]
    )

    # TimerActions to delay the launch of other nodes until `resource_manager` is initialized
    delayed_path_planner_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[path_planner_node]
    )

    delayed_resource_allocator_node = TimerAction(
        period=5.0,
        actions=[resource_allocator_node]
    )

    delayed_command_to_fastapi_service_node = TimerAction(
        period=5.0,
        actions=[command_to_fastapi_service_node]
    )

    # Assemble Launch Description
    ld = LaunchDescription()

    # Add Launch Arguments
    ld.add_action(sliding_window_size_arg)
    ld.add_action(navigate_url_arg)
    ld.add_action(status_url_arg)
    ld.add_action(distance_threshold_arg)

    # Add Nodes to Launch Description
    ld.add_action(resource_manager_node)
    ld.add_action(delayed_path_planner_node)
    ld.add_action(delayed_resource_allocator_node)
    ld.add_action(delayed_command_to_fastapi_service_node)

    return ld
