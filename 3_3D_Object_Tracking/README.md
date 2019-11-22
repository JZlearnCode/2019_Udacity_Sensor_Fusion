# SFND 3D Object Tracking

## Implementation 
Step 1. Match 3D bounding box objects
Takes previous and the current data frames and provides as outputs the ids of the matched bounding box.
Matches are the ones with the highest numbre of keypoint correspondences within the region of interest (bounding box)
between previous and the current frames. 

Step 2. Compute Lidar-based time-to-collision


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
