# path_visualizer.py

import rclpy
from rclpy.node import Node
from dram_interface.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)

        # Subscriptions to robot paths
        self.full_path_sub = self.create_subscription(Path, '/robot_paths', self.full_path_callback, 10)
        self.window_path_sub = self.create_subscription(Path, '/robot_window_paths', self.window_path_callback, 10)

        # Dictionaries to store markers
        self.full_path_markers = {}
        self.window_path_markers = {}

        self.get_logger().info("Path Visualizer Node Initialized")

    def full_path_callback(self, msg: Path):
        """
        Callback for full computed paths.
        Visualizes the full path as a green line.
        """
        robot_name = msg.robot_name
        task_id = msg.task_id

        # Create or update the full path marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = robot_name
        marker.id = 0  # ID 0 for full path
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Line width

        # Green color for full path
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for waypoint in msg.path:
            point = Point()
            point.x = waypoint.x
            point.y = waypoint.y
            point.z = 0.0  # Assuming 2D map
            marker.points.append(point)

        self.full_path_markers[robot_name] = marker

        self.publish_markers()

    def window_path_callback(self, msg: Path):
        """
        Callback for windowed paths.
        Visualizes the windowed path as a blue line.
        """
        robot_name = msg.robot_name
        task_id = msg.task_id

        # Create or update the window path marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = robot_name
        marker.id = 1  # ID 1 for window path
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width

        # Blue color for window path
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.points = []
        for waypoint in msg.path:
            point = Point()
            point.x = waypoint.x
            point.y = waypoint.y
            point.z = 0.0  # Assuming 2D map
            marker.points.append(point)

        self.window_path_markers[robot_name] = marker

        self.publish_markers()

    def publish_markers(self):
        """
        Publishes all markers to the MarkerArray.
        """
        marker_array = MarkerArray()

        # Add full path markers
        for marker in self.full_path_markers.values():
            marker_array.markers.append(marker)

        # Add window path markers
        for marker in self.window_path_markers.values():
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().debug("Published MarkerArray with full and window paths.")

    def on_shutdown(self):
        """
        Clean up markers upon shutdown.
        """
        marker_array = MarkerArray()

        # Delete all full path markers
        for robot_name in self.full_path_markers.keys():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = robot_name
            marker.id = 0
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        # Delete all window path markers
        for robot_name in self.window_path_markers.keys():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = robot_name
            marker.id = 1
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("Deleted all visualization markers.")


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()

    # Register shutdown callback to clean up markers
    node.get_logger().info("Path Visualizer Node Running. Press Ctrl+C to exit.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path Visualizer Node shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
