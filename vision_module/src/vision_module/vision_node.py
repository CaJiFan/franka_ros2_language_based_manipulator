import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_module.yolo_detector import YoloDetector
from vision_module.publishers.object_publisher import ObjectPublisher

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.yolo_detector = YoloDetector()
        self.object_publisher = ObjectPublisher(self)
        
        self.subscription = self.create_subscription(
            Image,
            'image_input',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        image = self.convert_image(msg)
        detected_objects = self.yolo_detector.detect_objects(image)
        self.object_publisher.publish(detected_objects)

    def convert_image(self, msg):
        # Convert the ROS Image message to a format suitable for YOLO
        # This function should handle the conversion logic
        pass

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()