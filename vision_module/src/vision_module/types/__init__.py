class DetectedObject:
    def __init__(self, label, confidence, position):
        self.label = label
        self.confidence = confidence
        self.position = position

__all__ = ['DetectedObject']