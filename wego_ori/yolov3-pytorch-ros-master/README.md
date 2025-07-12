# yolov3_pytorch_ros
ROS package for running yolov3 with USB_CAM_node

## Table of contents
  - [Environment](#Environment)
  - [Installation](#Installation)
  - [Usb cam setting](#camera-device-check)
  - [Using custom dataset](#custom-dataset)
  - [Running yolov3](#running-yolo-with-ros)

## Environment
> - Ubuntu 18.04, Python 2.7, torch=1.4.0
> ### Before you use this repository, we recommend you to install CUDA toolkit and cudnn
> - Python 2.7 and torch 1.4.0, the recommended compatible version of CUDA toolkit is  10.x (10.0, 10.1 or 10.2) and cudnn is 7.6.5

## Installation
> Download the required packages by 
> ```bash
> # In your catkin_ws/src/
> git clone https://github.com/hjinnkim/yolov3-pytorch-ros.git
> cd yolov3_pytorch_ros/weights
> sh download_weights.sh
> ```
> and go to your catkin_ws and catkin_make


> Also, you need **cv_bridge** packages.
> ```bash
> sudo apt install ros-melodic-cv-bridge
> ```
> rospy code needs execution previliege.
> ```bash
> roscd yolov3-ros/src/
> chmod +x yolov3_ros.py
> ```

## Custom Dataset
> If you have your custom trained weights, place the **weights file** in the *weights* directory and place the **.names** file in the *data* directory.
> Change the parameters in *launch* directory according to your files.

## Running YOLO with ROS
> You can run the yolov3 by:
> ```bash
> roslaunch yolov3-ros yolov3-[corresponding file].launch
> ```
> The *yolov3_node* will publish two topics:
> ```shell
> /yolov3_detect/compressed
> /yolov3_detect/objects
> ```
> You can see the detected images by
> ```shell
> rqt_image_view
> ```
