import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, OpaqueFunction

# List of pickup points and drop-off points
pickup_points = [f'r{i+1}' for i in range(100)]
dropoff_points = [f'd{i+1}' for i in range(5)]

# Global variable to keep track of the current dropoff point index
current_dropoff_index = 0

def generate_patrol_task(context):
    global current_dropoff_index

    # Select a random start point
    start_point = random.choice(pickup_points)
    
    # Select the next dropoff point in the cycle
    dropoff_point = dropoff_points[current_dropoff_index]

    # Update the dropoff point index for the next task, cycling back to 0 if it exceeds the length of dropoff_points
    current_dropoff_index = (current_dropoff_index + 1) % len(dropoff_points)

    return [
        Node(
            package='rmf_demos_tasks',
            executable='dispatch_patrol',
            name='dispatch_patrol',
            output='screen',
            arguments=[
                '-p', start_point, dropoff_point, start_point,
                '-n', '1',
            ]
        )
    ]

def repeat_patrol_task(context):
    return [
        TimerAction(
            period=20.0,
            actions=[
                OpaqueFunction(function=generate_patrol_task),
                OpaqueFunction(function=repeat_patrol_task)
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=repeat_patrol_task)
    ])

if __name__ == '__main__':
    generate_launch_description()
