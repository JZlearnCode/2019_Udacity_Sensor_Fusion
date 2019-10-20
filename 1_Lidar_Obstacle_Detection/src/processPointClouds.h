// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>
#include "render/box.h"

template <typename PointT>
class ProcessPointClouds {
 public:
  // constructor
  ProcessPointClouds();
  // deconstructor
  ~ProcessPointClouds();

  // Downsample point cloud by voxel grid filtering and region of interest
  // filtering
  // filter_resolution: resolution of the point cloud after filter
  // min_point: defines the minimum position of the bounding box region
  // max_point: defines the maximum position of the bounding box region
  typename pcl::PointCloud<PointT>::Ptr FilterCloud(
      typename pcl::PointCloud<PointT>::Ptr cloud, float filter_resolution,
      Eigen::Vector4f min_point, Eigen::Vector4f max_point);

  // Separate point cloud into ground and non-ground. The non-ground portion
  // contains the obstacles
  // ground_point_indices: indices for points belong to the ground
  // cloud:   input point cloud
  // return a pair of point clouds: first is non-ground, second is ground
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
  SeparateClouds(std::unordered_set<int>& ground_point_indices,
                 typename pcl::PointCloud<PointT>::Ptr cloud);

  // cloud: input point cloud
  // max_iterations: maximum number of iterations to perform.
  // distance_tolerance:
  // point with distance to current plane model < distance_tolerance will be
  // counted as inlier.
  // return indices for points belong to the ground.
  std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                                   int max_iterations,
                                   float distance_tolerance);

  // Euclidean Clustering
  // cloud: input point cloud to be clustered into smaller clusters
  // cluster_tolerance: points with distance larger than cluster_tolerance would
  // belong to two different clusters
  // min_size: minimum size of a cluster
  // max_size: maximum size of a cluster
  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
      typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
      int min_size, int max_size);

  // BoundingBox looks at the min and max point values of an input cloud and
  // stores thoes parameters in a box struct container
  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  // Read point cloud data from pcd file
  typename pcl::PointCloud<PointT>::Ptr LoadPcd(std::string file);

  // Obtain paths to all pcd files under directory data_path
  std::vector<boost::filesystem::path> StreamPcd(std::string data_path);

 private:
  // Downsample point cloud
  // Voxel grid filtering creates a cubic grid and filter the cloud by only
  // leaving a single point per voxel cube, so the larger the cube length
  // the lower the resolution of the point cloud
  // cloud: input point cloud
  // filter_resolution: resolution of the point cloud after filter
  // cloud_filtered: downsampled point cloud
  void VoxelFilter(typename pcl::PointCloud<PointT>::Ptr cloud,
                   float filter_resolution,
                   typename pcl::PointCloud<PointT>::Ptr cloud_filtered);

  // Remove points too far from car outside a bounding box,
  // or too close to car roof
  // cloud: input point cloud
  // min_point: defines the minimum position of the bounding box region
  // max_point: defines the maximum position of the bounding box region
  void RegionOfInterestFilter(
      typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f min_point,
      Eigen::Vector4f max_point,
      typename pcl::PointCloud<PointT>::Ptr cloud_filtered);
};
#endif /* PROCESSPOINTCLOUDS_H_ */