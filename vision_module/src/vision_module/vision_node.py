import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_module.yolo_detector import YoloDetector
from vision_module.publishers.object_publisher import ObjectPublisher

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.yolo_detector = YoloDetector()
        self.object_publisher = ObjectPublisher(self)
        self.bridge = CvBridge()

        # subscribe to 'input/image' to match launch remapping in launch file
        self.subscription = self.create_subscription(
            Image,
            'input/image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        image = self.convert_image(msg)
        if image is None:
            return
        detected_objects = self.yolo_detector.detect_objects(image)
        self.object_publisher.publish(detected_objects)

    def convert_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            return cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()