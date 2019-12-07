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

3. TTC calculation using different combinations of feature detectors and descriptors.
TTC for camera and lidar used here are averaged over the frames, instead of TTC of a single frame.
TTC difference is calculated by averaging the difference of camera and lidar over frames. 
Thus, some of the entries such as "ORB" detector and "FREAK" descriptor, TTC difference is larger than the absolute difference of TTC camera and TTC lidar in the table. 

TTC lidar in theses experiments are 11.2 s. 
The result marked as NA are the ones with large fluctuation of result, even had infinity value in certain fraems.

| Detector  |  Descriptor    | TTC camera (s) | TTC difference (s) | 
| ----------|----------------|----------------|--------------------|
| SHITOMASI |   BRIEF        |       11.9     |      2.17                         
| SHITOMASI |   ORB          |       11.3     |      1.8     
| SHITOMASI |   FREAK        |       10.7     |      1.9  
| SHITOMASI |   SIFT         |       12.8     |      1.8
| HARRIS    |all descriptors |       NA       |      NA 
| FAST      |   BRIEF        |       12.7     |      1.5  
| FAST      |   ORB          |       12.1     |      2     
| FAST      |   FREAK        |       10.1     |      0.8      
| FAST      |   SIFT         |       13.1     |      1.8       
| BRISK     |   BRIEF        |       16.7     |      5  
| BRISK     |   ORB          |       13.8     |      2.7        
| BRISK     |   FREAK        |       13.6     |      2.9     
| BRISK     |   SIFT         |       16.3     |      5.4        
| ORB       |   BRIEF        |       56.2     |      51.9     
| ORB       |   ORB          |       40.3     |      80.6        
| ORB       |   FREAK        |       6.2      |      23.7       
| ORB       |   SIFT         |       15.1     |      7.9       
| AKAZE     |   AKAZE        |       14.2     |      2.8        
| SIFT      |   BRIEF        |       11.5     |      0.8     
| SIFT      |   ORB          |       Out of Memory
| SIFT      |   FREAK        |       11.3     |      0.9       
| SIFT      |   SIFT         |       10.6     |      1.1         |


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
