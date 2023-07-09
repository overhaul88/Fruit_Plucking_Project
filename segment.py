import cv2
import numpy
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor


cap = cv2.VideoCapture(0)
model = YOLO(r'D:\TS\MangoDetection\MRCNN\models\v8_02.pt')

while True:
    ret, color_frame = cap.read()
    results = model.predict(color_frame, show=False)
    for result in results:
        boxes = result.boxes
        boxez = boxes.xywh
        boxez = boxez.cpu().numpy()
        for box in boxez:
            x, y, w, h = box
            point = (int(x),int(y))
            cv2.rectangle(color_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (36,255,12), 2)

    cv2.imshow('color frame', color_frame)
    key = cv2.waitKey(1)
    if key ==27:
        break 

