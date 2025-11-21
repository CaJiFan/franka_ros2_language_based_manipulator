from pathlib import Path
import unittest
from vision_module.yolo_detector import YoloDetector

class TestYoloDetector(unittest.TestCase):

    def setUp(self):
        tests_dir = Path(__file__).resolve().parent
        model_path = tests_dir / 'model' / 'yolo.weights'          # adjust filename to your model
        image_path = tests_dir / 'img' / 'test_img.jpg'       # adjust filename to your test image
        if not model_path.exists() or not image_path.exists():
            self.skipTest(f"Missing test assets: {model_path} or {image_path}. Place them under tests/img or update paths.")
        self.detector = YoloDetector()
        self.detector.load_model(str(model_path))
        self.test_image = str(image_path)

    def test_load_model(self):
        self.assertIsNotNone(self.detector.model)

    def test_detect_objects(self):
        detections = self.detector.detect_objects(self.test_image)
        self.assertIsInstance(detections, list)

    def test_get_positions(self):
        detections = self.detector.detect_objects(self.test_image)
        positions = self.detector.get_positions(detections)
        self.assertIsInstance(positions, list)

if __name__ == '__main__':
    unittest.main()