
from ultralytics import YOLO


# 1. Load YOLOv8 model
model = YOLO("/home/rahgirrafi/Gesture_Controlled_RoboCar/saved_models/4/best (2).pt")
model.export(format='tflite', int8=True, data="hand-keypoints.yaml" ,imgsz=640, half= True, nms= True, batch= 1, fraction =1, device = 'cpu')

