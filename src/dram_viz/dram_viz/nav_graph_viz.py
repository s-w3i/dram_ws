import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from rmf_building_map_msgs.msg import Graph


class NavGraphVisualizer(Node):
    def __init__(self):
        super().__init__('nav_graph_visualizer')

        # QoS profile for transient local durability
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Topic to subscribe for navigation graph
        self.create_subscription(Graph, 'nav_graphs', self.graph_callback, 10)

        # Topic to publish markers for RViz with transient local durability
        self.marker_pub = self.create_publisher(MarkerArray, 'nav_graph_markers', qos)

        self.published_once = False  # Ensure one-time publishing
        self.get_logger().info("NavGraphVisualizer Node initialized")

    def graph_callback(self, msg: Graph):
        if self.published_once:
            return

        marker_array = MarkerArray()

        # Create markers for vertices
        for i, vertex in enumerate(msg.vertices):
            vertex_marker = Marker()
            vertex_marker.header.frame_id = "map"
            vertex_marker.header.stamp = self.get_clock().now().to_msg()
            vertex_marker.ns = "vertices"
            vertex_marker.id = i
            vertex_marker.type = Marker.CUBE
            vertex_marker.action = Marker.ADD
            vertex_marker.pose.position.x = vertex.x
            vertex_marker.pose.position.y = vertex.y
            vertex_marker.pose.position.z = 0.0
            vertex_marker.pose.orientation.x = 0.0
            vertex_marker.pose.orientation.y = 0.0
            vertex_marker.pose.orientation.z = 0.0
            vertex_marker.pose.orientation.w = 1.0
            vertex_marker.scale.x = 0.5  # Size of the square block
            vertex_marker.scale.y = 0.5
            vertex_marker.scale.z = 0.1
            vertex_marker.color.r = 1.0  # Orange color
            vertex_marker.color.g = 0.5
            vertex_marker.color.b = 0.0
            vertex_marker.color.a = 1.0  # Fully opaque
            marker_array.markers.append(vertex_marker)

        # Create markers for edges
        edge_id = len(msg.vertices)  # Ensure unique IDs for edges
        for edge in msg.edges:
            edge_marker = Marker()
            edge_marker.header.frame_id = "map"
            edge_marker.header.stamp = self.get_clock().now().to_msg()
            edge_marker.ns = "edges"
            edge_marker.id = edge_id
            edge_marker.type = Marker.LINE_STRIP
            edge_marker.action = Marker.ADD
            edge_marker.scale.x = 0.2  # Line width
            edge_marker.color.r = 1.0  # Orange color
            edge_marker.color.g = 0.5
            edge_marker.color.b = 0.0
            edge_marker.color.a = 1.0  # Fully opaque

            # Add the start and end points of the edge
            start_vertex = msg.vertices[edge.v1_idx]
            end_vertex = msg.vertices[edge.v2_idx]

            edge_marker.points = [
                self.create_point(start_vertex.x, start_vertex.y),
                self.create_point(end_vertex.x, end_vertex.y)
            ]
            marker_array.markers.append(edge_marker)
            edge_id += 1

        # Publish the marker array
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published navigation graph markers")

        self.published_once = True  # Mark as published and stop further updates
        self.get_logger().info("NavGraphVisualizer has published and will not republish.")

    @staticmethod
    def create_point(x, y, z=0.0):
        from geometry_msgs.msg import Point
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main(args=None):
    rclpy.init(args=args)
    node = NavGraphVisualizer()
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
