# resource_manager.py

import rclpy
from rclpy.node import Node
from dram_interface.msg import GraphNodes  # Import the custom GraphNodes message
from rmf_building_map_msgs.msg import Graph, GraphNode
from dram_interface.srv import UpdateResources
import threading
from math import sqrt


class ResourceManager(Node):
    def __init__(self):
        super().__init__('resource_manager')

        # Subscribers
        self.nav_graph_sub = self.create_subscription(
            Graph,
            '/nav_graphs',
            self.nav_graph_callback,
            10
        )

        # Publishers
        self.available_resources_pub = self.create_publisher(
            GraphNodes,
            '/available_resources',
            10
        )

        # Publisher for reserved nodes
        self.reserved_nodes_pub = self.create_publisher(
            GraphNodes,
            '/reserved_nodes',
            10
        )

        # Service Server
        self.update_resources_srv = self.create_service(
            UpdateResources,
            'update_resources',
            self.update_resources_callback
        )

        # Internal Data Structures
        self.full_graph = None  # Complete navigation graph received from /nav_graphs
        self.robot_reserved_nodes = {}  # {robot_name: set of (x, y) tuples}

        # Threading lock to ensure thread safety
        self.lock = threading.Lock()

        self.get_logger().info("Resource Manager Node Initialized")

    def nav_graph_callback(self, msg: Graph):
        """
        Callback to receive the full navigation graph. Only initializes the full graph once.
        """
        with self.lock:
            if self.full_graph is None:
                self.full_graph = msg
                self.get_logger().info(f"Received navigation graph with {len(msg.vertices)} vertices and {len(msg.edges)} edges.")

                # Initially, all resources are available
                self.publish_available_resources()
            else:
                pass  # Ignore subsequent graph updates

    def update_resources_callback(self, request, response):
        """
        Service callback to update reserved resources based on new allocations.
        If windowed_path.path is empty, it releases all resources reserved by the robot.
        Otherwise, it reserves resources based on the provided graph_nodes.
        """
        with self.lock:
            robot_name = request.robot_name

            if not self.full_graph:
                response.success = False
                response.message = "Navigation graph not received."
                self.get_logger().error(response.message)
                return response

            if not request.windowed_path.path:
                # Empty path signifies resource release
                if robot_name in self.robot_reserved_nodes:
                    del self.robot_reserved_nodes[robot_name]
                self.get_logger().info(f"Released all resources for robot {robot_name}.")
            else:
                # Reserve new nodes based on request.graph_nodes
                # Extract (x, y) tuples from GraphNode messages
                reserved_coords = set()
                for node in request.graph_nodes:
                    coord = (node.x, node.y)
                    reserved_coords.add(coord)
                self.robot_reserved_nodes[robot_name] = reserved_coords
                self.get_logger().info(f"Reserved {len(reserved_coords)} nodes for robot {robot_name}.")

            # Recompute available resources
            self.publish_available_resources()

            response.success = True
            response.message = f"Resources updated successfully for robot {robot_name}."
            return response

    def publish_available_resources(self):
        """
        Computes and publishes the available (unreserved) resources.
        """
        if not self.full_graph:
            self.get_logger().error("Navigation graph not available. Cannot publish available resources.")
            return

        # Collect all reserved node coordinates
        total_reserved_coords = set()
        for reserved_coords in self.robot_reserved_nodes.values():
            total_reserved_coords.update(reserved_coords)

        # Debug: Log reserved node coordinates
        self.get_logger().debug(f"Total Reserved Coordinates: {total_reserved_coords}")
        self.get_logger().debug(f"Number of Reserved Nodes: {len(total_reserved_coords)}")

        # Compute available nodes by excluding reserved coordinates within threshold
        available_nodes = [
            node for node in self.full_graph.vertices
            if not any(self.are_coords_close((node.x, node.y), coord) for coord in total_reserved_coords)
        ]

        # Debug: Log available node coordinates
        available_node_coords = set((node.x, node.y) for node in available_nodes)
        self.get_logger().debug(f"Available Node Coordinates: {available_node_coords}")
        self.get_logger().debug(f"Number of Available Nodes: {len(available_nodes)}")

        # Create and publish the available resources message
        available_resources_msg = GraphNodes()
        available_resources_msg.nodes = available_nodes
        self.available_resources_pub.publish(available_resources_msg)
        self.get_logger().info(f"Published available resources: {len(available_nodes)} nodes.")

        # Create and publish the reserved nodes message
        total_reserved_nodes = self.find_nodes_by_coordinates(total_reserved_coords)
        reserved_nodes_msg = GraphNodes()
        reserved_nodes_msg.nodes = total_reserved_nodes
        self.reserved_nodes_pub.publish(reserved_nodes_msg)
        self.get_logger().info(f"Published reserved nodes: {len(total_reserved_nodes)} nodes.")

    def find_nodes_by_coordinates(self, coords_set, threshold=0.01):
        """
        Finds and returns GraphNode messages that match the given set of (x, y) coordinates within a threshold.
        """
        matching_nodes = []
        for node in self.full_graph.vertices:
            for coord in coords_set:
                if self.are_coords_close((node.x, node.y), coord, threshold):
                    matching_nodes.append(node)
                    break  # Avoid duplicate entries
        return matching_nodes

    def are_coords_close(self, coord1, coord2, threshold=0.01):
        """
        Determines if two coordinates are close enough based on the threshold.
        """
        return sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2) <= threshold


def main(args=None):
    rclpy.init(args=args)
    node = ResourceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Resource Manager Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
