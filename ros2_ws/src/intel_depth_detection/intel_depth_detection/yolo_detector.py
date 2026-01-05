from ultralytics import YOLO
import numpy as np
import torch

class YoloDetector:
    def __init__(self, model_path='yolov11n.pt'):
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path).to(device)

    def detect(self, image, conf_threshold=0.4):
        results = self.model(image, conf=conf_threshold, verbose=False)
        detections = []

        for box in results[0].boxes:
            if int(box.cls[0]) != 0:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append({
                'x_min': x1,
                'y_min': y1,
                'x_max': x2,
                'y_max': y2,
                'confidence': float(box.conf[0])
            })
        return detections
