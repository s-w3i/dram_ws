import rclpy
from rclpy.node import Node
from dram_interface.msg import Path, Location  # Custom messages for path publishing
from rmf_fleet_msgs.msg import RobotState  # Correct RobotState import
import requests
from math import sqrt


class CommandToFastAPIService(Node):
    def __init__(self):
        super().__init__('command_to_fastapi_service')

        # Declare ROS2 parameters for the FastAPI URLs and other configurations
        self.declare_parameter('navigate_url', 'http://127.0.0.1:22011/open-rmf/rmf_demos_fm/navigate/')
        self.declare_parameter('status_url', 'http://127.0.0.1:22011/open-rmf/rmf_demos_fm/status/')
        self.declare_parameter('distance_threshold', 1.0)  # Meters

        # Get the FastAPI URLs and distance threshold
        self.navigate_url = self.get_parameter('navigate_url').get_parameter_value().string_value
        self.status_url = self.get_parameter('status_url').get_parameter_value().string_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value

        # Subscribe to the /robot_window_paths topic
        self.create_subscription(Path, '/robot_window_paths', self.robot_paths_callback, 10)

        # Subscribe to the /robot_state topic
        self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)

        # Timer to periodically check and send commands
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

        # Track commands for each robot
        self.robot_commands = {}  # {robot_name: {path, current_index, cmd_id, waiting}}

        # Track robot statuses
        self.robot_status = {}  # {robot_name: task_id, location}

        self.get_logger().info(f"Navigate URL set to {self.navigate_url}")
        self.get_logger().info(f"Status URL set to {self.status_url}")
        self.get_logger().info(f"Distance threshold set to {self.distance_threshold} meters")

    def robot_paths_callback(self, msg: Path):
        """
        Callback for the /robot_window_paths topic to receive and store robot paths.
        """
        robot_name = msg.robot_name
        path = msg.path

        if not path:
            self.get_logger().warning(f"No path available for robot {robot_name}.")
            return

        # Retrieve existing command data if available
        existing_command_data = self.robot_commands.get(robot_name, {})

        # Preserve cmd_id if it exists, else start from 0
        cmd_id = existing_command_data.get('cmd_id', 0)

        # Initialize or update the robot's command data
        self.robot_commands[robot_name] = {
            'path': path,
            'current_index': 0,
            'cmd_id': cmd_id,
            'waiting': False
        }

        self.get_logger().info(f"Received new path for robot {robot_name} with {len(path)} waypoints.")

    def robot_state_callback(self, msg: RobotState):
        """
        Callback for the /robot_state topic to update robot statuses.
        """
        robot_name = msg.name
        task_id = int(msg.task_id) if msg.task_id else -1  # Convert to int, handle empty string
        current_location = (msg.location.x, msg.location.y)

        self.robot_status[robot_name] = {
            'task_id': task_id,
            'location': current_location
        }

        self.get_logger().debug(f"Updated status for {robot_name}: task_id={task_id}, location={current_location}")

    def timer_callback(self):
        """
        Timer callback to process and send commands based on robot statuses.
        """
        for robot_name, command_data in self.robot_commands.items():
            if command_data['current_index'] >= len(command_data['path']):
                #self.get_logger().info(f"{robot_name} waiting for new commands.")
                continue

            if not command_data['waiting']:
                # Send the next waypoint
                self.send_next_waypoint(robot_name)
            else:
                # Check if the robot has reached the current waypoint
                if robot_name in self.robot_status:
                    robot_loc = self.robot_status[robot_name]['location']
                    current_waypoint = command_data['path'][command_data['current_index']]
                    waypoint_loc = (current_waypoint.x, current_waypoint.y)

                    distance = self.calculate_distance(robot_loc, waypoint_loc)
                    self.get_logger().debug(f"Robot {robot_name} distance to waypoint {command_data['current_index']}: {distance:.2f} meters.")

                    if distance <= self.distance_threshold:
                        self.get_logger().info(f"Robot {robot_name} has reached waypoint {command_data['current_index']} (cmd_id={command_data['cmd_id']}).")
                        command_data['waiting'] = False
                        command_data['current_index'] += 1
                        # Removed: command_data['cmd_id'] += 1
                    else:
                        self.get_logger().debug(f"Robot {robot_name} has not yet reached waypoint {command_data['current_index']}.")

    def send_next_waypoint(self, robot_name):
        """
        Sends the next waypoint command to the FastAPI server for the specified robot.
        """
        command_data = self.robot_commands[robot_name]
        current_index = command_data['current_index']
        cmd_id = command_data['cmd_id']
        waypoint = command_data['path'][current_index]

        destination = {
            "x": waypoint.x,
            "y": waypoint.y,
            "yaw": waypoint.yaw
        }

        payload = {
            "map_name": waypoint.level_name,
            "task": "navigate",
            "destination": destination,
            "data": {},  # Add additional data if needed
            "speed_limit": 0,
            "toggle": True
        }

        # Append query parameters to the URL
        full_url = f"{self.navigate_url}?robot_name={robot_name}&cmd_id={cmd_id}"

        self.get_logger().info(f"Sending cmd_id {cmd_id} to robot {robot_name} for destination {destination}.")

        try:
            response = requests.post(full_url, json=payload, timeout=5)
            if response.status_code == 200:
                response_data = response.json()
                if response_data.get('success'):
                    self.get_logger().info(f"Command cmd_id {cmd_id} sent successfully to robot {robot_name}.")
                    command_data['waiting'] = True
                    command_data['cmd_id'] += 1  # Increment cmd_id here
                else:
                    self.get_logger().warning(f"Failed to send command cmd_id {cmd_id} to robot {robot_name}: {response_data.get('msg')}")
            else:
                self.get_logger().error(f"Failed to send command to FastAPI: HTTP {response.status_code}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error sending command to FastAPI for robot {robot_name}: {e}")

    @staticmethod
    def calculate_distance(loc1, loc2):
        """
        Calculates Euclidean distance between two (x, y) locations.
        """
        return sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = CommandToFastAPIService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()