import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rmf_fleet_msgs.msg import RobotState

class RobotStateVisualizer(Node):
    def __init__(self):
        super().__init__('robot_state_visualizer')

        # Subscriber for robot state
        self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10)

        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, 'robot_state_markers', 10)

        self.get_logger().info("RobotStateVisualizer Node initialized")

    def robot_state_callback(self, msg: RobotState):
        # Extract robot location
        robot_name = msg.name
        robot_location = msg.location

        # Create a marker for this robot
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_states"
        marker.id = hash(robot_name) % (10 ** 6)  # Unique ID based on robot name
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position from the robot state
        marker.pose.position.x = robot_location.x
        marker.pose.position.y = robot_location.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Marker size
        marker.scale.x = 0.5  # Diameter of the sphere
        marker.scale.y = 0.5
        marker.scale.z = 0.3

        # Marker color: purple
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0  # Fully opaque

        # Publish the marker
        self.marker_pub.publish(marker)
        #self.get_logger().info(f"Updated marker for robot '{robot_name}' at ({robot_location.x}, {robot_location.y})")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateVisualizer()
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
