# resource_allocator.py

import rclpy
from rclpy.node import Node
from dram_interface.srv import ComputePath, MoveRobot, UpdateResources
from dram_interface.msg import Path, Location, WindowGraphNodes  # Import the new WindowGraphNodes message
from rmf_building_map_msgs.msg import GraphNode
from rmf_fleet_msgs.msg import RobotState
from math import sqrt
import uuid


class ResourceAllocator(Node):
    def __init__(self):
        super().__init__('resource_allocator')

        # Parameters
        self.declare_parameter('sliding_window_size', 3)
        self.sliding_window_size = self.get_parameter('sliding_window_size').get_parameter_value().integer_value

        # Publishers
        self.window_path_pub = self.create_publisher(Path, '/robot_window_paths', 10)
        self.window_graph_nodes_pub = self.create_publisher(WindowGraphNodes, '/robot_window_graph_nodes', 10)  # New publisher

        # Subscribers
        self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)

        # Services
        self.create_service(MoveRobot, '/move_robot', self.move_robot_callback)

        # Clients
        self.path_compute_client = self.create_client(ComputePath, '/compute_path')
        self.update_resources_client = self.create_client(UpdateResources, '/update_resources')  # Ensure the service name is correct

        # Internal Data Structures
        self.robot_full_paths = {}        # {robot_name: Path}
        self.robot_graph_nodes = {}       # {robot_name: [GraphNode]}
        self.robot_window_indices = {}    # {robot_name: start_index}
        self.robot_goals = {}             # {robot_name: goal_vertex_name}

        self.get_logger().info(f"Resource Allocator Node Initialized with sliding_window_size = {self.sliding_window_size}")

    def move_robot_callback(self, request, response):
        """
        Callback to handle movement commands for robots.
        """
        robot_name = request.robot_name
        goal_vertex = request.goal_vertex_name

        self.get_logger().info(f"Received command to move {robot_name} to {goal_vertex}")

        # Store the goal
        self.robot_goals[robot_name] = goal_vertex

        # Request path computation
        self.request_path_computation(robot_name, goal_vertex)

        response.success = True
        response.message = f"Command received to move {robot_name} to {goal_vertex}"
        return response

    def request_path_computation(self, robot_name, goal_vertex):
        """
        Requests the path planner to compute a path for the robot to the goal vertex.
        """
        if not self.path_compute_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Path planner service not available.')
            return

        request = ComputePath.Request()
        request.robot_name = robot_name
        request.goal_vertex_name = goal_vertex
        future = self.path_compute_client.call_async(request)
        future.add_done_callback(lambda fut: self.path_computation_callback(fut, robot_name))

    def path_computation_callback(self, future, robot_name):
        """
        Callback when the path computation service returns.
        """
        try:
            response = future.result()
            if response.success:
                path = response.path
                graph_nodes = response.graph_nodes  # List of GraphNode messages
                self.robot_full_paths[robot_name] = path
                self.robot_graph_nodes[robot_name] = graph_nodes
                self.robot_window_indices[robot_name] = 0
                self.get_logger().info(f"Received full path for robot {robot_name} with {len(path.path)} waypoints and {len(graph_nodes)} GraphNodes.")

                # Publish the initial window
                self.publish_windowed_path(robot_name)

                # Update resources via service
                self.update_resources(robot_name, self.get_current_windowed_path(robot_name))
            else:
                self.get_logger().error(f"Path computation failed for robot {robot_name}: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def update_resources(self, robot_name, windowed_path):
        """
        Calls the resource_manager service to update reserved resources.
        """
        if not self.update_resources_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Resource Manager service not available.')
            return

        request = UpdateResources.Request()
        request.robot_name = robot_name
        request.windowed_path = windowed_path  # Correct assignment (entire Path message)
        request.graph_nodes = self.get_windowed_graph_nodes(robot_name, windowed_path)  # Pass corresponding GraphNodes

        future = self.update_resources_client.call_async(request)
        future.add_done_callback(lambda fut: self.update_resources_callback(fut, robot_name))

    def update_resources_callback(self, future, robot_name):
        """
        Callback for the resource_manager service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Resource Manager updated resources for robot {robot_name}.")
            else:
                self.get_logger().error(f"Failed to update resources for robot {robot_name}: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call to Resource Manager failed: {e}")

    def robot_state_callback(self, msg: RobotState):
        """
        Callback for receiving the robot's current state.
        Determines if the robot has passed the first waypoint in its current window.
        If so, slides the window forward and publishes the next set of waypoints.
        Also detects if the robot has reached its destination to release resources.
        """
        robot_name = msg.name
        current_location = (msg.location.x, msg.location.y)

        if robot_name not in self.robot_full_paths:
            return

        window_start_idx = self.robot_window_indices.get(robot_name, 0)
        full_path = self.robot_full_paths[robot_name]
        full_graph_nodes = self.robot_graph_nodes.get(robot_name, [])

        if window_start_idx >= len(full_path.path):
            # Path already completed; ensure resources are released
            self.get_logger().info(f"Robot {robot_name} has already completed its path. Ensuring resources are released.")
            self.release_resources(robot_name)
            return

        # Get the first waypoint in the current window
        first_waypoint = full_path.path[window_start_idx]
        first_waypoint_loc = (first_waypoint.x, first_waypoint.y)

        distance = self.calculate_distance(current_location, first_waypoint_loc)
        self.get_logger().debug(f"Robot {robot_name} distance to waypoint {window_start_idx}: {distance:.2f} meters.")

        if distance <= 0.5:  # Threshold to consider waypoint as passed
            self.get_logger().info(f"Robot {robot_name} has passed waypoint {window_start_idx}. Sliding window forward.")
            self.robot_window_indices[robot_name] += 1

            # Check if the robot has reached the end of its path
            if self.robot_window_indices[robot_name] >= len(full_path.path):
                self.get_logger().info(f"Robot {robot_name} has reached its destination. Releasing resources.")
                self.release_resources(robot_name)
                return

            # Publish the new windowed path
            self.publish_windowed_path(robot_name)

            # Get the updated windowed path
            new_window_path = self.get_current_windowed_path(robot_name)

            # Update resources via service
            self.update_resources(robot_name, new_window_path)

    def publish_windowed_path(self, robot_name):
        """
        Publishes a windowed path segment based on the sliding window size.
        Includes both Path and corresponding GraphNodes.
        """
        if robot_name not in self.robot_full_paths or robot_name not in self.robot_graph_nodes:
            self.get_logger().error(f"No full path or GraphNodes found for robot {robot_name}. Cannot publish windowed path.")
            return

        full_path = self.robot_full_paths[robot_name]
        full_graph_nodes = self.robot_graph_nodes[robot_name]
        start_idx = self.robot_window_indices.get(robot_name, 0)
        end_idx = min(start_idx + self.sliding_window_size, len(full_path.path))

        if start_idx >= len(full_path.path):
            self.get_logger().info(f"Robot {robot_name} has reached the end of its path.")
            return

        window_path = Path()
        window_path.robot_name = robot_name
        window_path.fleet_name = full_path.fleet_name
        window_path.task_id = f"window_{robot_name}_{uuid.uuid4()}"

        window_graph_nodes = []

        for idx in range(start_idx, end_idx):
            waypoint = full_path.path[idx]
            location = Location()
            location.x = waypoint.x
            location.y = waypoint.y
            location.yaw = waypoint.yaw
            location.level_name = waypoint.level_name
            window_path.path.append(location)

            # Collect the corresponding GraphNode
            graph_node = full_graph_nodes[idx]
            window_graph_nodes.append(graph_node)

        # Publish the windowed path for visualization
        self.window_path_pub.publish(window_path)
        self.get_logger().info(f"Published windowed path for robot {robot_name}: waypoints {start_idx} to {end_idx - 1}.")

        # Publish the windowed GraphNodes
        self.publish_windowed_graph_nodes(robot_name, window_graph_nodes)

    def publish_windowed_graph_nodes(self, robot_name, graph_nodes):
        """
        Publishes the windowed GraphNodes for the robot.
        """
        window_graph_nodes_msg = WindowGraphNodes()
        window_graph_nodes_msg.robot_name = robot_name
        window_graph_nodes_msg.graph_nodes = graph_nodes

        self.window_graph_nodes_pub.publish(window_graph_nodes_msg)
        self.get_logger().info(f"Published windowed GraphNodes for robot {robot_name}: {len(graph_nodes)} nodes.")

    def get_windowed_graph_nodes(self, robot_name, windowed_path):
        """
        Retrieves the GraphNode messages corresponding to the windowed path.
        """
        full_graph_nodes = self.robot_graph_nodes.get(robot_name, [])
        window_start_idx = self.robot_window_indices.get(robot_name, 0)
        window_end_idx = min(window_start_idx + self.sliding_window_size, len(full_graph_nodes))
        return full_graph_nodes[window_start_idx:window_end_idx]

    def get_current_windowed_path(self, robot_name):
        """
        Constructs the current windowed path for the robot based on the sliding window.
        Returns a Path message containing the windowed path.
        """
        if robot_name not in self.robot_full_paths or robot_name not in self.robot_graph_nodes:
            self.get_logger().error(f"No full path or GraphNodes found for robot {robot_name}.")
            return Path()

        full_path = self.robot_full_paths[robot_name]
        full_graph_nodes = self.robot_graph_nodes[robot_name]
        start_idx = self.robot_window_indices.get(robot_name, 0)
        end_idx = min(start_idx + self.sliding_window_size, len(full_path.path))

        window_path = Path()
        window_path.robot_name = robot_name
        window_path.fleet_name = full_path.fleet_name
        window_path.task_id = f"window_{robot_name}_{uuid.uuid4()}"

        for idx in range(start_idx, end_idx):
            waypoint = full_path.path[idx]
            location = Location()
            location.x = waypoint.x
            location.y = waypoint.y
            location.yaw = waypoint.yaw
            location.level_name = waypoint.level_name
            window_path.path.append(location)

        return window_path

    def release_resources(self, robot_name):
        """
        Calls the resource_manager service to release all reserved resources for the robot.
        """
        if not self.update_resources_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Resource Manager service not available. Cannot release resources.')
            return

        request = UpdateResources.Request()
        request.robot_name = robot_name
        request.windowed_path = Path()      # Empty path signifies resource release
        request.graph_nodes = []            # Empty list signifies resource release

        future = self.update_resources_client.call_async(request)
        future.add_done_callback(lambda fut: self.release_resources_callback(fut, robot_name))

    def release_resources_callback(self, future, robot_name):
        """
        Callback for the resource_manager service response when releasing resources.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Resource Manager released resources for robot {robot_name}.")
            else:
                self.get_logger().error(f"Failed to release resources for robot {robot_name}: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call to Resource Manager failed while releasing resources: {e}")

    @staticmethod
    def calculate_distance(loc1, loc2):
        """
        Calculates Euclidean distance between two (x, y) locations.
        """
        return sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Resource Allocator Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
