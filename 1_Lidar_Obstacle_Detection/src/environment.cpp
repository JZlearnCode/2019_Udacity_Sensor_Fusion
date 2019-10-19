/*
Load lidar point cloud and perform object detection.
Author: Jin Zhu
*/
#include "processPointClouds.cpp"
#include "processPointClouds.h"
#include "render/render.h"

// Preprocess and detect obstacles in a point cloud
void RunDetector(pcl::visualization::PCLVisualizer::Ptr& viewer,
                 ProcessPointClouds<pcl::PointXYZI>* point_processor,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud) {
  // Step 1. Downsample point cloud by voxel grid filtering and region of
  // interest filtering
  // Define resolution for voxel grid filtering
  float grid_resolution = 0.4;
  // Defines min and max position of region of interest
  Eigen::Vector4f min_point = Eigen::Vector4f(-10, -6, -2, 1);
  Eigen::Vector4f max_point = Eigen::Vector4f(30, 6, 1, 1);

  input_cloud = point_processor->FilterCloud(input_cloud, grid_resolution,
                                             min_point, max_point);

  // Step 2. Remove ground plane using RANSAC
  int max_iterations = 40;
  float distance_tolerance = 0.3;
  std::unordered_set<int> inliers = point_processor->Ransac3D(
      input_cloud, max_iterations, distance_tolerance);

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segment_cloud = point_processor->SeparateClouds(inliers, input_cloud);

  renderPointCloud(viewer, segment_cloud.first, "obstacleCloud",
                   Color(1, 0, 0));
  renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));

  // Step 3.Obstacle detection using KD tree and euclidean clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters =
      point_processor->Clustering(segment_cloud.first, 0.5, 10, 150);
  // Step 4. Display detection result by giving a bounding box for each obstacle
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacle_clusters) {
    renderPointCloud(viewer, cluster, "obsCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = point_processor->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

int main() {
  // Initialize viewer and set camera at 45 degree angle view
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Object Detector"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  int distance = 16;
  viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);

  // Initialize point processor
  ProcessPointClouds<pcl::PointXYZI> point_processor;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
  // stream is a vector of chronologically ordered vector pcd files
  std::vector<boost::filesystem::path> stream =
      point_processor.streamPcd("../data/pcd_example");
  auto streamIterator = stream.begin();

  // Run detection
  while (!viewer->wasStopped()) {
    // Clear Viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // Load pcd and run obstacle detector
    input_cloud = point_processor.loadPcd((*streamIterator).string());
    RunDetector(viewer, &point_processor, input_cloud);
    streamIterator++;
    // loop back to the first file if finished
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }
    viewer->spinOnce();
  }
  return 0;
}