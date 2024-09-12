# Fruit Plucking Robot - Project Overview

## Introduction

This project focuses on the development of a **Fruit Plucking Robot** for detecting and plucking mangoes autonomously. It integrates computer vision software with autonomous control systems using a Husky A200 robot, equipped with depth-sensing technology and running on ROS (Robot Operating System).

The project was led by **Syed Adeel Ahmad** (IIT Kanpur) under the guidance of **Dr. Tushar Sandhan** from the Department of Electrical Engineering at IIT Kanpur, in collaboration with the **Ministry of Electronics and Information Technology**.

## Objectives

1. **Mango Tree and Fruit Detection**: Developing visual perception software for detecting mango trees and fruits.
2. **Depth-Sensing**: Utilizing a depth camera to acquire real-world coordinates of detected mangoes.
3. **Autonomous Integration**: Integrating the detection software with the Husky robot for autonomous control using ROS.

## Specifications

### Depth Camera

- **Model**: Intel Realsense D455
- **Firmware**: 5.15.0.2
- **SDK**: Realsense SDK 2.54.1
- **Connection**: USB 3.2

### Husky Robot

- **Model**: Husky A200
- **Controller**: Nvidia Jetson AGX Xavier
- **Operating System**: Ubuntu 20.04 (L4T 35.2.1)
- **ROS Version**: Noetic
- **Network IP**: 192.168.1.133

## Progress Updates

### 1. Mango Tree and Fruit Detection

#### 1.1 Dataset Preparation
A dataset of 580 unique images of mango trees and fruits was collected from various campus locations. Augmentation techniques, including Gaussian blur, exposure variation, cutouts, and mosaicing, were applied to ensure diversity in the dataset.

#### 1.2 Algorithm Development
Custom algorithms were developed for generating PNG masks from annotated images, and the dataset was formatted for Mask-RCNN processing.

#### 1.3 Model Training and Evaluation
Using **PyTorch**, models such as **MaskRCNN**, **Yolov5**, and **Yolov8-Segmentation** were trained. The Yolov8-Segmentation model achieved the best results:
- **mAP50-95**: 0.856 for mango fruits, 0.63 for mango trees
- **Inference Time**: 45.8 ms

#### Model Comparison:

| Model             | Accuracy (mAP50-95) | Inference Time |
|-------------------|---------------------|----------------|
| MaskRCNN          | 0.773               | 384 ms         |
| Yolov5 Medium      | 0.631               | 43.7 ms        |
| Yolov8-seg Medium  | 0.856               | 45.8 ms        |

### 2. ROS Integration for Autonomous Control

- **Visual Data Capture**: The Realsense depth camera was used for capturing and processing visual data.
- **Coordinate Generation**: Real-world coordinates of mangoes were derived from the detection pipeline.
- **ROS Package**: A ROS package was created with a publisher node that takes camera input, performs detections, and publishes velocity commands for the Husky robot.

### 3. Future Work

- **Accuracy Improvement**: Further model refinement and integration with a robotic arm for mango plucking.
- **Field Testing**: Extensive testing in varied real-world environments, lighting, and weather conditions.

## Resources

- **Project Code**: [Github Repository](https://github.com/overhaul88/Fruit_Plucking_Project/tree/master)
- **Dataset Download**: [Dataset](https://drive.google.com/file/d/1B8NenJiQRKJCQHA9sIPbmC0wjwO94v-d/view?usp=sharing)
- **Demonstration Video**: [Husky Robot Demonstration](https://drive.google.com/file/d/1KmfDyEdjHGefiauMt7N8SRAHTIPreHWM/view?usp=sharing)

---

### Contact Information

- **Syed Adeel Ahmad** (Project Lead): [syed21@iitk.ac.in](mailto:syed21@iitk.ac.in)
- **Dr. Prem Raj Kala** (Project Guide): [praj@cse.iitk.ac.in](mailto:praj@cse.iitk.ac.in)

---

This project is a part of the **Ministry of Electronics and Information Technology** initiative at **IIT Kanpur**.

