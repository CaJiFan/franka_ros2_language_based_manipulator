from pathlib import Path
import unittest
# from src.vision_module.yolo_detector import YoloDetector
from src.vision_module.yolo_detector_ultralytics import YOLODetectorUltralytics

class TestYoloDetector(unittest.TestCase):

    def setUp(self):
        tests_dir = Path(__file__).resolve().parent
        model_path = tests_dir / 'model' / 'yoloe-11m-seg.pt'          # adjust filename to your model
        image_path = tests_dir / 'img' / 'test_img.jpg'       # adjust filename to your test image
        if not image_path.exists():
            self.skipTest(f"Missing test assets: {model_path} or {image_path}. Place them under tests/img or update paths.")
        self.detector = YOLODetectorUltralytics(model_name=model_path)
        # self.detector.load_model(str(model_path))
        self.names = ["cube", "toy"]
        self.detector.set_classes(self.names, self.detector.get_text_pe(self.names))
        self.test_image = str(image_path)

    def test_load_model(self):
        self.assertIsNotNone(self.detector.model)

    def test_detect_objects(self):
        detections = self.detector.detect_objects(self.test_image)
        self.assertIsInstance(detections, list)

    def test_get_positions(self):
        detections = self.detector.detect_objects(self.test_image)
        positions = self.detector.get_positions(detections)
        print('positions', positions)
        self.assertIsInstance(positions, list)

if __name__ == '__main__':
    unittest.main()