# SFND 3D Object Tracking

## Intro
This project estimates the time to collision (TTC) using both camera and Lidar results. YOLO was used for object detection and outputs bounding boxes. Bounding box combined with keypoints detected were used for TTC estimation from image. Bounding box, camera Lidar calibration, and Lidar point cloud were used for calculating TTC using Lidar. 

<img src="images/readme_images/example_final_result.png"/>
The image above is an example result. The TTC calculated using Lidar and camera are printed on top. A YOLO detection for car is plotted as bounding box, and the dots inside are Lidar points projected on camera image.

The average absolute difference between TTC estimated using camera and Lidar is 1.4 s. 

## File structure
```
project
│   README.md
└───src
|   └───src/            
|        |camFusion.cpp         ttc calculation using camera and lidar 
|        |inputOutputUtil.cpp   read data and visualization  
|        |lidarData.cpp         process and visualization of lidar data  
|        |matching2D.cpp        keypoint detection + matching for camera  
|        |objectDetection2D.cpp YOLO object detection  
|        |main.cpp              main function   
|   └───libs/                   header files
└───dat                         Yolo model 2D image and 3D lidar points 
```
## Implementation 
Following is a flow chart showing how camera and Lidar data are used for TTC estimation.
<img src="images/readme_images/flowchart.png"/>

Step 1. Object Detection Using YOLO
Objects with low confidence score are removed, and non-maximum-supression was performed. 
<img src="images/readme_images/YOLO_result.png"/>

Step 2. 
Lidar points whose projection into the camera falls into the same bounding box are grouped together. Lidar points that belong to only one bounding box are kept to avoid ambiguity. 

Step 3. Compute Camera-based time-to-collision
3.1 Keypoint detection and generate keypoint descriptors 
3.2 Associate bounding box with corresponding 2D feature points.
3.3 Compute camera TC
<img src="images/readme_images/cameraTTCVariables.png"/>
<img src="images/readme_images/cameraTTCFormula.png"/>

Step 4. Match bounding boxes 
Takes previous and the current data frames and provides as outputs the ids of the matched bounding box.
The bounding box B in current frame is a best match for the bounding box A in previous frame if B has the most matched keypoints with keypoints in A, compared with other bounding boxes in current frame. 

Step 5. Compute Lidar-based time-to-collision
Lidar points beloning to the vehicle in front of ego vehicle in the ego line
are used for computing ttc. 
<img src="images/readme_images/lidarTTCvariables.png"/>
<img src="images/readme_images/lidarTTCformula.png"/>

Since the bounding boxes do not always reflect the true vehicle dimensions
and the aspect ratio differs between images, using bounding box height or width for TTC computation would thus lead to significant estimation erros.

Height ratio h1/h0 is replaced by dk/dk' in the image below. 
<img src="images/readme_images/cameraTTCkeypointDist.png"/>

## Performance evaluation
1. The TTC calculated from Lidar and camera here are plausible, since multiple 
techniques were adopted to remove outliers.

2. The SIFT algorithm was chosen create keypoint detector and descriptor 
based on the evaluation result from the previous project.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
