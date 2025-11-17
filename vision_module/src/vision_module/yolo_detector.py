class YoloDetector:
    def __init__(self, model_path, confidence_threshold=0.5):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = self.load_model()

    def load_model(self):
        # Load the YOLO model from the specified path
        import cv2
        net = cv2.dnn.readNet(self.model_path)
        return net

    def detect_objects(self, image):
        # Process the image and detect objects using YOLO
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
                        'label': class_id,
                        'confidence': confidence,
                        'position': (center_x, center_y, w, h)
                    })
        return positions