import rclpy
from rclpy.node import Node
from rmf_building_map_msgs.msg import Graph, GraphNode, GraphEdge, Param
import yaml
import os

class YAMLGraphPublisher(Node):
    def __init__(self):
        super().__init__('yaml_graph_publisher')

        # Declare and get the 'nav_graph_file' parameter
        self.declare_parameter('nav_graph_file', '')
        self.yaml_file_path = self.get_parameter('nav_graph_file').get_parameter_value().string_value

        if not self.yaml_file_path or not os.path.isfile(self.yaml_file_path):
            self.get_logger().error(f"Invalid or missing YAML file: {self.yaml_file_path}")
            raise FileNotFoundError(f"YAML file not found: {self.yaml_file_path}")

        self.publisher_ = self.create_publisher(Graph, '/nav_graphs', 1)
        self.timer = self.create_timer(1.0, self.publish_graph)
        self.get_logger().info(f"YAML Graph Publisher Node initialized with file: {self.yaml_file_path}")

    def read_yaml_file(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML file: {e}")
            return None

    def publish_graph(self):
        map_data = self.read_yaml_file()
        if map_data is None:
            self.get_logger().error('Map data could not be loaded.')
            return

        level_name = list(map_data['levels'].keys())[0]
        level_data = map_data['levels'][level_name]

        graph_msg = Graph()
        graph_msg.name = level_name

        # Parse vertices
        for idx, vertex in enumerate(level_data['vertices']):
            node_msg = GraphNode()
            node_msg.x = float(vertex[0])
            node_msg.y = float(vertex[1])
            node_msg.name = vertex[2].get('name', '')

            # Add parameters
            for key, value in vertex[2].items():
                if key == 'name':
                    continue
                param_msg = Param()
                param_msg.name = key
                param_msg.type = self.get_param_type(value)
                if isinstance(value, int):
                    param_msg.value_int = value
                elif isinstance(value, float):
                    param_msg.value_float = value
                elif isinstance(value, str):
                    param_msg.value_string = value
                elif isinstance(value, bool):
                    param_msg.value_bool = value
                node_msg.params.append(param_msg)

            graph_msg.vertices.append(node_msg)

        # Parse edges
        for edge in level_data['lanes']:
            edge_msg = GraphEdge()
            edge_msg.v1_idx = edge[0]
            edge_msg.v2_idx = edge[1]
            edge_msg.edge_type = GraphEdge.EDGE_TYPE_BIDIRECTIONAL

            # Add parameters (if any)
            for key, value in edge[2].items():
                param_msg = Param()
                param_msg.name = key
                param_msg.type = self.get_param_type(value)
                if isinstance(value, int):
                    param_msg.value_int = value
                elif isinstance(value, float):
                    param_msg.value_float = value
                elif isinstance(value, str):
                    param_msg.value_string = value
                elif isinstance(value, bool):
                    param_msg.value_bool = value
                edge_msg.params.append(param_msg)

            graph_msg.edges.append(edge_msg)

        # Publish the graph message
        self.publisher_.publish(graph_msg)
        #self.get_logger().info(f'Published graph: {graph_msg.name}')

    @staticmethod
    def get_param_type(value):
        if isinstance(value, int):
            return Param.TYPE_INT
        elif isinstance(value, float):
            return Param.TYPE_DOUBLE
        elif isinstance(value, str):
            return Param.TYPE_STRING
        elif isinstance(value, bool):
            return Param.TYPE_BOOL
        return Param.TYPE_UNDEFINED


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YAMLGraphPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()