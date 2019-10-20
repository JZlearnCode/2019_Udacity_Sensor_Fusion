# Sensor Fusion Self-Driving Car Course
<img src="media/detection.gif"/>

# Run on Ubuntu 
```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/JZlearnCode/2019_Udacity_Sensor_Fusion.git
$> cd 1_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

# Implementation
## Step 1. Downsample point cloud by voxel grid filtering and region of interest filtering
Filtering took 7 ms per frame 
### Voxel grid filtering
Before vs after 
### Region of interest filtering 
Before vs after 

Speed of preprocessing:  
## Step 2. Remove ground plane using RANSAC
#### RANSAC 
#### Plane math 
Speed of RANSAC: fps
Speed of removing ground: fps
## Step 3. Obstacle detection using KD tree and Euclidean Clustering
#### KD tree
#### Euclidean Clustering  
Speed of Euclidean clustering: fps  

# File structure
```
project
│   README.md
└───src
|   └───src/            Implementation of lidar obstacle detection
|        └───render/    Render and visualize 
|        └───sensors/   Define simulation of lidar sensor  
│        |   environment.cpp    Main function to load point cloud data and run detection
|        |   processPointClouds.cpp/.h   Functions to manipulate point cloud data 
└───data 
|   └───pcd_example/*.pcd            Point cloud data     
```

 

