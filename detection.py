import cv2
import numpy
import pyrealsense2
from realsense_depth import *
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor


# cap = cv2.VideoCapture(0)
dc = DepthCamera()
model = YOLO(r'D:\TS\MangoDetection\MRCNN\models\v8_02.pt')
# model = YOLO('yolov8m.pt')

# img = cv2.imread("D:/Python/ultralytics/testing/cntr01.jpg")

while True:
    # ret, color_frame = cap.read()
    ret, depth_frame, color_frame = dc.get_frame()
    results = model.predict(color_frame, show=False)
    for result in results:
        boxes = result.boxes
        boxez = boxes.xywh
        boxez = boxez.cpu().numpy()
        for box in boxez:
            x, y, w, h = box
            point = (int(x),int(y))
            distance = depth_frame[point[1], point[0]]
            # print(distance)
            cv2.rectangle(color_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (36,255,12), 2)
            cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1]-10), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)

    cv2.imshow('color frame', color_frame)
    key = cv2.waitKey(1)
    if key ==27:
        break 

