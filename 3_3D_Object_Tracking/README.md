# SFND 3D Object Tracking

## Intro

<img src="images/readme_images/example_final_result.png"/>
The average absolute difference between TTC estimated using camera and Lidar is 1.39661 s. 

## File structure
```
project
│   README.md
└───src
|   └───src/            Implementation 
└───dat                 Yolo model 
└───images              2D image and 3D lidar points 
```
# TODO
showLidarImgOverlay     math to project Lidar points to camera image

## Implementation 

Step 1. Object Detection Using YOLO
<img src="images/readme_images/YOLO_result.png"/>

Step 2. 
Lidar points in the ego line are shown below. 
Lidar points whose projection into the camera falls into the same bounding box are grouped together. 
<img src="images/readme_images/lidar_points_view.png"/>

Step 1. Match 3D bounding box objects
Takes previous and the current data frames and provides as outputs the ids of the matched bounding box.
Matches are the ones with the highest numbre of keypoint correspondences within the region of interest (bounding box)
between previous and the current frames. 

Step 2. Compute Lidar-based time-to-collision
<img src="images/readme_images/lidarTTCvariables.png"/>
<img src="images/readme_images/lidarTTCformula.png"/>

Step 3. Compute Camera-based time-to-collision
3.1 Associate bounding box with corresponding 2D feature points.
3.2 Compute TTC
<img src="images/readme_images/cameraTTCVariables.png"/>
<img src="images/readme_images/cameraTTCFormula.png"/>

Since the bounding boxes do not always reflect the true vehicle dimensions
and the aspect ratio differs between images, using bounding box height or width for TTC computation would thus lead to significant estimation erros.

Height ratio h1/h0 is replaced by dk/dk' in the image below. 

<img src="images/readme_images/cameraTTCkeypointDist.png"/>

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
