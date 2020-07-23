# Autonomous Car Navigation &amp; Control through Computer Vision and Sensor Fusion

## Description
The project design consists of three main blocks. The Computer Vision (CV) Module analyzes the car surroundings through a camera to detect traffic signs, traffic lights, vehicles, and human beings, then make decisions accordingly. The Navigation Module consists of a navigation unit and an obstacle detection unit. The navigation unit is responsible for collecting data from sensors to get the current state of the car, like GPS coordination, orientation, and speed. The obstacle detection unit assures that the vehicle avoids hitting obstacles encountered along its path. The Power and Driving Module is responsible for powering the electronics, alongside powering and controlling the acceleration and steering motors.

The CV Module is written in C++ because it is much faster than Python in processing the inputs for this heavy task, while other modules are written in Python that even when slow can still work fast for their tasks. The CV module accomplishes object detection for traffic signs using pre-trained HOG-SVM models. The code used for training and generating the models can be found [here](https://github.com/opencv/opencv/blob/master/samples/cpp/train_HOG.cpp). 

This project uses ROS as a way of communication between the three modules.

## Test
### The project was optimized and tested on this system:
1. Computer Vision:
   - Raspberry Pi 4 (4 GB)
   - Raspberry Pi Camera Module V2
2. Navigation:
   - BerryGPS-IMU V3
3. Obstacle Detection:
   - RPLiDAR A1
#### The Computer Vision devices can be changed given that the replacement computer can be interfaced with the two other modules and has OpenCV and ROS installed.
