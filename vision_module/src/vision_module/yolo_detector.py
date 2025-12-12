import cv2
import numpy as np

class YoloDetector:
    def __init__(self, model_path=None, confidence_threshold=0.5):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None

    def load_model(self, model_path=None):
        if model_path:
            self.model_path = model_path
        if not self.model_path:
            raise ValueError("No model path specified for YOLO model")
        net = cv2.dnn.readNet(self.model_path)
        self.model = net
        return self.model

    def detect_objects(self, image):
        if self.model is None:
            raise RuntimeError("Model not loaded. Call load_model(path) first.")
        # allow passing image path or ndarray
        if isinstance(image, str):
            image = cv2.imread(image)
        if image is None:
            return []
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.model.setInput(blob)
        layer_names = self.model.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.model.getUnconnectedOutLayers()]
        detections = self.model.forward(output_layers)
        return self.get_positions(detections, width, height)

    def get_positions(self, detections, width, height):
        positions = []
        for output in detections:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    positions.append({
                        'label': int(class_id),
                        'confidence': float(confidence),
                        'position': (center_x, center_y, w, h)
                    })
        return positions