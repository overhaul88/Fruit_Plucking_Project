#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
import cv2 as cv
import pyrealsense2 as rs
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor

model = YOLO('/home/adeel/catkin_ws/src/mango_detection/src/scripts/Fruit_Plucking_Project/models/Trees_v8_03.pt')

def detect_tree(color_frame):
    cw = color_frame.shape[1] / 2
    ch = color_frame.shape[0] / 2
    coord = [cw, ch]
    results = model.predict(color_frame, show=False)
    for result in results:
        boxes = result.boxes
        boxez = boxes.xywh
        boxez = boxez.cpu().numpy()
        for box in boxez:
            x, y, w, h = box
            coord = [x,y]
            return coord    

def main():
    rospy.init_node('marker_pose_node')
    pub = rospy.Publisher('/vector', Float64MultiArray, queue_size=10)
    rospy.loginfo('the node is working')
    rate = rospy.Rate(30)
    bridge = CvBridge()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    pipe=pipeline.start(config)

    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    params = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, params)

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    print(depth_intrinsics)
    

    while not rospy.is_shutdown():
        frame = pipeline.wait_for_frames()
        color_get = frame.get_color_frame()
        depth_get = frame.get_depth_frame()
        color_data = np.asanyarray(color_get.get_data())
        depth_data = np.asanyarray(depth_get.get_data())
        if not color_get or not depth_get:
            continue
        image = cv.cvtColor(color_data, cv.COLOR_BGR2RGB)
        corner, id, rejected_markers = detector.detectMarkers(image)
        cv.aruco.drawDetectedMarkers(image, corner,id, (0,255,0) )
        coord = (detect_tree(image))
        depth = depth_get.as_depth_frame().get_distance(coord[0],coord[1])
        result = rs.rs2_deproject_pixel_to_point(depth_intrinsics, coord, depth)
        pub_text = Float64MultiArray()
        pub_text.data = result
        pub.publish(pub_text)
        print(result)
        cv.circle(image, coord, 5, (0,0,255), thickness = -1)
        cv.imshow('video', image)
        cv.waitKey(1)
        rate.sleep()
    
    
    pipeline.stop()


if __name__ == '__main__':
    main()