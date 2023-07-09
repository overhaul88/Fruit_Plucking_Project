#!/usr/bin/env python3

import rospy
import cv2
import numpy
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor

class Camera_node:
	def __init__(self):
		rospy.init_node('camera', anonymous=False)
		self.vid = cv2.VideoCapture(0)
		self.model = YOLO('/home/adeel/catkin_ws/src/mango_detection/src/scripts/Fruit_Plucking_Project/models/v8_04.pt')
		
	def detections(self):
		while(True):
			ret, color_frame = self.vid.read()
			results = self.model.predict(color_frame, show=False)
			for result in results:
				boxes = result.boxes
				boxez = boxes.xywh
				boxez = boxez.cpu().numpy()
				for box in boxez:
					x, y, w, h = box
					# point = (int(x),int(y))
					cv2.rectangle(color_frame, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (36,255,12), 2)
			cv2.imshow('frame', color_frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			
		self.vid.release()
		cv2.destroyAllWindows()
		
if __name__ == '__main__':
	cn = Camera_node()
	r = rospy.Rate(60)
	while not rospy.is_shutdown():
		cn.detections()
		r.sleep()
