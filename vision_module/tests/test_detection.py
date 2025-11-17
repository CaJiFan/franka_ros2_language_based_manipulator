import unittest
from vision_module.yolo_detector import YoloDetector

class TestYoloDetector(unittest.TestCase):

    def setUp(self):
        self.detector = YoloDetector()
        self.detector.load_model('path/to/yolo/model')

    def test_load_model(self):
        self.assertIsNotNone(self.detector.model)

    def test_detect_objects(self):
        test_image = 'path/to/test/image.jpg'
        detections = self.detector.detect_objects(test_image)
        self.assertIsInstance(detections, list)

    def test_get_positions(self):
        test_image = 'path/to/test/image.jpg'
        detections = self.detector.detect_objects(test_image)
        positions = self.detector.get_positions(detections)
        self.assertIsInstance(positions, list)

if __name__ == '__main__':
    unittest.main()