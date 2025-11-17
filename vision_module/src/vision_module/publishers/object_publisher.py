from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

class ObjectPublisher(Node):
    def __init__(self):
        super().__init__('object_publisher')
        self.publisher = self.create_publisher(String, 'detected_objects', 10)

    def publish(self, object_info):
        msg = String()
        msg.data = object_info
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')