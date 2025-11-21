from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
import json

class ObjectPublisher:
    def __init__(self, node):
        # use the provided node to create publisher so everything uses the same ROS node
        self._node = node
        self._publisher = node.create_publisher(String, 'detected_objects', 10)

    def publish(self, object_info):
        msg = String()
        # serialize object_info (list/dict) to JSON
        try:
            msg.data = json.dumps(object_info)
        except Exception:
            msg.data = str(object_info)
        self._publisher.publish(msg)
        self._node.get_logger().info(f'Published: {msg.data}')