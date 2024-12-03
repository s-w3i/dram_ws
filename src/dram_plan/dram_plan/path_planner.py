import rclpy
from rclpy.node import Node
from dram_interface.msg import Path, Location
from rmf_building_map_msgs.msg import Graph, GraphNode
from rmf_fleet_msgs.msg import RobotState
from dram_interface.srv import ComputePath
from math import sqrt
import heapq
import uuid


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Publishers
        self.robot_path_pub = self.create_publisher(
            Path,
            '/robot_paths',
            10
        )

        # Subscribers
        self.nav_graph_sub = self.create_subscription(
            Graph,
            '/nav_graphs',
            self.nav_graph_callback,
            10
        )
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # Service
        self.compute_path_service = self.create_service(
            ComputePath,
            '/compute_path',
            self.compute_path_callback
        )

        # Internal Data Structures
        self.nav_graph = None  # Full navigation graph from /nav_graph
        self.robot_states = {}  # {robot_name: (x, y)}

        # Retry parameters
        self.max_retries = 5          # Maximum number of retries
        self.retry_delay = 2.0        # Delay between retries in seconds

        # Pending requests: {robot_name: {'request': ..., 'response': ..., 'attempt': ..., 'timer': Timer}}
        self.pending_requests = {}

        self.get_logger().info("Path Planner Node Initialized")

    def nav_graph_callback(self, msg: Graph):
        """
        Callback to receive the full navigation graph.
        """
        self.nav_graph = msg
        self.get_logger().info(f"Received navigation graph with {len(msg.vertices)} vertices and {len(msg.edges)} edges.")

    def robot_state_callback(self, msg: RobotState):
        """
        Callback to receive robot states.
        Updates the robot's current position.
        """
        self.robot_states[msg.name] = (msg.location.x, msg.location.y)
        self.get_logger().debug(f"Updated state for robot {msg.name}: ({msg.location.x}, {msg.location.y})")

    def compute_path_callback(self, request, response):
        """
        Service to compute path for a robot, considering available resources.
        Uses a non-blocking retry mechanism to handle unavailable resources.
        """
        robot_name = request.robot_name

        if robot_name in self.pending_requests:
            response.success = False
            response.message = f"Path computation already in progress for robot {robot_name}."
            self.get_logger().error(response.message)
            return response

        # Initialize the pending request
        self.pending_requests[robot_name] = {
            'request': request,
            'response': response,
            'attempt': 0,
            'timer': None
        }

        # Start the first attempt
        self.attempt_path_computation(robot_name)

        # Return response now; it will be updated asynchronously
        return response

    def attempt_path_computation(self, robot_name):
        """
        Attempts to compute a path for the given robot. Retries if necessary.
        """
        if robot_name not in self.pending_requests:
            # This might occur if the request was already handled
            self.get_logger().debug(f"No pending request found for robot {robot_name}. Ignoring retry.")
            return

        request = self.pending_requests[robot_name]['request']
        response = self.pending_requests[robot_name]['response']
        attempt = self.pending_requests[robot_name]['attempt']

        self.get_logger().info(f"Attempting path computation for robot {robot_name}. Attempt {attempt + 1}/{self.max_retries}.")

        # Increment attempt count
        self.pending_requests[robot_name]['attempt'] += 1

        # Check if nav_graph is ready
        if not self.nav_graph:
            response.success = False
            response.message = "Navigation graph not received."
            self.get_logger().error(response.message)
            self.cleanup_pending_request(robot_name)
            return

        # Check robot state
        if robot_name not in self.robot_states:
            response.success = False
            response.message = f"No state available for robot: {robot_name}"
            self.get_logger().error(response.message)
            self.cleanup_pending_request(robot_name)
            return

        # Find the goal vertex index in the nav_graph
        goal_idx = self.find_vertex_index_by_name(request.goal_vertex_name, self.nav_graph)
        if goal_idx is None:
            response.success = False
            response.message = f"Goal vertex '{request.goal_vertex_name}' not found in the navigation graph."
            self.get_logger().error(response.message)
            self.cleanup_pending_request(robot_name)
            return

        # Find the closest node to the robot's current position
        current_pos = self.robot_states[robot_name]
        start_idx = self.find_closest_node(current_pos, self.nav_graph)

        if start_idx is None:
            response.success = False
            response.message = f"No available starting node near robot {robot_name}'s current position."
            self.get_logger().error(response.message)
            self.cleanup_pending_request(robot_name)
            return

        # Compute the shortest path using Dijkstra's algorithm on the nav_graph
        path_indices = self.compute_shortest_path(start_idx, goal_idx, self.nav_graph)

        if path_indices:
            # Path found; construct and return the path
            path_msg = Path()
            graph_nodes = []  # List to hold corresponding GraphNode messages
            path_msg.robot_name = robot_name
            path_msg.fleet_name = "v1"  # Replace with your fleet name
            path_msg.task_id = f"task_{robot_name}_{uuid.uuid4()}"

            for idx in path_indices:
                node = self.nav_graph.vertices[idx]
                location = Location()
                location.x = node.x
                location.y = node.y
                location.yaw = 0.0  # Assuming no yaw; modify if necessary
                location.level_name = self.nav_graph.name  # Assuming all nodes are on the same level
                path_msg.path.append(location)

                # Collect the corresponding GraphNode
                graph_nodes.append(node)

            # Publish the full path for visualization
            self.robot_path_pub.publish(path_msg)
            self.get_logger().info(f"Published full path for robot {robot_name} with task_id {path_msg.task_id}")

            # Populate the service response
            response.success = True
            response.message = "Path computed successfully."
            response.path = path_msg
            response.graph_nodes = graph_nodes  # Assuming graph_nodes is added to ComputePath.srv

            # Cleanup and remove the pending request
            self.cleanup_pending_request(robot_name)
        else:
            # No path found; check if we should retry
            if attempt < self.max_retries:
                self.get_logger().warning(
                    f"No available path found for robot {robot_name}. Attempt {attempt + 1}/{self.max_retries}. Retrying in {self.retry_delay} seconds..."
                )
                # Schedule the next attempt using a timer
                timer = self.create_timer(
                    self.retry_delay,
                    lambda robot=robot_name: self.attempt_path_computation(robot)
                )
                self.pending_requests[robot_name]['timer'] = timer
            else:
                # Max retries reached; return failure response
                response.success = False
                response.message = f"Failed to compute path for robot {robot_name} after {self.max_retries} attempts."
                self.get_logger().error(response.message)
                self.cleanup_pending_request(robot_name)

    def cleanup_pending_request(self, robot_name):
        """
        Cleans up the pending request for the given robot, including canceling timers.
        """
        if robot_name in self.pending_requests:
            # Cancel any active timer
            timer = self.pending_requests[robot_name].get('timer')
            if timer:
                timer.cancel()
            del self.pending_requests[robot_name]
            self.get_logger().info(f"Cleaned up pending request for robot {robot_name}.")

    def find_vertex_index_by_name(self, name, graph: Graph):
        """
        Finds the index of a vertex by its name in the given graph.
        """
        for idx, vertex in enumerate(graph.vertices):
            if vertex.name == name:
                return idx
        return None

    def find_closest_node(self, position, graph: Graph):
        """
        Finds the index of the closest node in the graph to the given position.
        """
        min_distance = float('inf')
        closest_idx = None
        for idx, node in enumerate(graph.vertices):
            distance = sqrt((position[0] - node.x) ** 2 + (position[1] - node.y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx
        return closest_idx

    def compute_shortest_path(self, start_idx, goal_idx, graph: Graph):
        """
        Computes the shortest path from start_idx to goal_idx using Dijkstra's algorithm.
        Considers the directionality of edges based on 'is_bidirectional' parameter.
        """
        vertices = graph.vertices
        edges = graph.edges

        # Build adjacency list from nav_graph considering edge directionality
        adjacency = {idx: [] for idx in range(len(vertices))}

        for edge in edges:
            # Add edge from v1 to v2
            adjacency[edge.v1_idx].append(edge.v2_idx)

            # Check if the edge is bidirectional
            is_bidirectional = True  # Default to True if not specified
            for param in edge.params:
                if param.name == 'is_bidirectional':
                    if param.type == 4:  # TYPE_BOOL
                        is_bidirectional = param.value_bool
                    elif param.type == 2:  # TYPE_INT
                        is_bidirectional = (param.value_int != 0)
                    elif param.type == 1:  # TYPE_STRING
                        is_bidirectional = (param.value_string.lower() == 'true')
                    else:
                        is_bidirectional = True  # Default to True
                    break  # Found the parameter, no need to check further

            if is_bidirectional:
                # Add edge from v2 to v1
                adjacency[edge.v2_idx].append(edge.v1_idx)

        # Dijkstra's algorithm
        queue = []
        heapq.heappush(queue, (0, start_idx, [start_idx]))
        visited = set()

        while queue:
            cost, current, path = heapq.heappop(queue)

            if current in visited:
                continue

            if current == goal_idx:
                return path

            visited.add(current)

            for neighbor in adjacency[current]:
                if neighbor in visited:
                    continue
                new_cost = cost + sqrt(
                    (vertices[current].x - vertices[neighbor].x) ** 2 +
                    (vertices[current].y - vertices[neighbor].y) ** 2
                )
                heapq.heappush(queue, (new_cost, neighbor, path + [neighbor]))

        return []  # No path found


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Path Planner Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
