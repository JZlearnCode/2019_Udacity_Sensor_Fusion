#include "processPointClouds.h"
#include <chrono>
#include <string>
#include <vector>
#include "kdtree.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::VoxelFilter(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filter_resolution,
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered) {
  // Create the filtering object: downsample the datset
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  // set resolution in x, y, z direction
  vg.setLeafSize(filter_resolution, filter_resolution, filter_resolution);
  vg.filter(*cloud_filtered);
}

template <typename PointT>
void ProcessPointClouds<PointT>::RegionOfInterestFilter(
    typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f min_point,
    Eigen::Vector4f max_point,
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered) {
  // CropBox(bool extract_removed_indices), set to true if you want to be able
  // to extarct the indices of points beling removed
  pcl::CropBox<PointT> region(true);
  region.setMin(min_point);
  region.setMax(max_point);
  region.setInputCloud(cloud);
  region.filter(*cloud_filtered);

  // remove the points caused by lidar ray hitting root of the car
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 0));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 0));
  roof.setInputCloud(cloud_filtered);
  // filter can return indices of points within the defined region (close to
  // roof of car)
  roof.filter(indices);
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices) {
    inliers->indices.push_back(point);
  }
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filter_resolution,
    Eigen::Vector4f min_point, Eigen::Vector4f max_point) {
  // Time segmentation process
  auto start_time = std::chrono::steady_clock::now();
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>);
  VoxelFilter(cloud, filter_resolution, cloud_filtered);
  // Region of interest filtering
  RegionOfInterestFilter(cloud_filtered, min_point, max_point, cloud_filtered);
  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);
  std::cout << "filtering took " << elapsed_time.count() << " milliseconds"
            << std::endl;
  return cloud_filtered;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distance_tolerance) {
  std::unordered_set<int> inliersResult;
  auto start_time = std::chrono::steady_clock::now();
  while (maxIterations--) {
    // Randomly sample 3 points and fit plane
    // use unordered_set to avoid picking the same point
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      inliers.insert(rand() % (cloud->points.size()));
    }
    std::vector<float> x, y, z;
    for (auto itr = inliers.begin(); itr != inliers.end(); itr++) {
      x.push_back(cloud->points[*itr].x);
      y.push_back(cloud->points[*itr].y);
      z.push_back(cloud->points[*itr].z);
    }

    float a = (y[1] - y[0]) * (z[2] - z[0]) - (z[1] - z[0]) * (y[2] - y[0]);
    float b = (z[1] - z[0]) * (x[2] - x[0]) - (x[1] - x[0]) * (z[2] - z[0]);
    float c = (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0]);
    float d = -(a * x[0] + b * y[0] + c * z[0]);
    float sqrt_abc = sqrt(a * a + b * b + c * c);
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    for (int index = 0; index < cloud->points.size(); index++) {
      // if the index is already in inlier (the 2 points that's originally
      // picked)
      if (inliers.count(index) > 0) continue;
      PointT p = cloud->points[index];
      // fabs: float abs, not using abs here to avoid rounding
      float dist = fabs(a * p.x + b * p.y + c * p.z + d) / sqrt_abc;
      if (dist <= distance_tolerance) inliers.insert(index);
      if (inliers.size() > inliersResult.size()) {
        inliersResult = inliers;
      }
    }
  }
  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);
  std::cout << "RANSAC took " << elapsed_time.count() << "milliseconds"
            << std::endl;
  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    std::unordered_set<int>& ground_point_indices,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];
    if (ground_point_indices.count(index))
      plane_cloud->points.push_back(point);
    else
      obstacle_cloud->points.push_back(point);
  }
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      separted_clouds(obstacle_cloud, plane_cloud);
  return separted_clouds;
}

template <typename PointT>
void ProcessPointClouds<PointT>::GrowCluster(
    int index, typename pcl::PointCloud<PointT>::Ptr cloud,
    std::vector<int>& cluster_idx, std::vector<bool>& processed, KdTree* tree,
    float distance_tolerance) {
  processed[index] = true;
  cluster_idx.push_back(index);

  std::vector<int> nearby_neighbors =
      tree->Search(cloud->points[index], distance_tolerance);
  for (int id : nearby_neighbors) {
    if (!processed[id]) {
      GrowCluster(id, cloud, cluster_idx, processed, tree, distance_tolerance);
    }
  }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::EuclideanClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
    int min_cluster_size, int max_cluster_size) {
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  KdTree* tree = new KdTree;
  int num_points = cloud->points.size();
  for (int i = 0; i < num_points; i++) {
    tree->InsertNode(cloud->points[i], i);
  }

  // keep track of which point has been processed or not
  std::vector<bool> processed(num_points, false);
  for (int i = 0; i < num_points; i++) {
    if (!processed[i]) {
      std::vector<int> cluster_idx;
      typename pcl::PointCloud<PointT>::Ptr cluster(
          new pcl::PointCloud<PointT>());
      GrowCluster(i, cloud, cluster_idx, processed, tree, cluster_tolerance);
      if (cluster_idx.size() >= min_cluster_size &&
          cluster_idx.size() <= max_cluster_size) {
        // create point cloud cluster from vector of point indices
        for (int j = 0; j < cluster_idx.size(); j++) {
          cluster->points.push_back(cloud->points[cluster_idx[j]]);
        }
        // save the cluster
        clusters.push_back(cluster);
      }
    }
  }
  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT min_point, max_point;
  pcl::getMinMax3D(*cluster, min_point, max_point);

  Box box;
  box.x_min = min_point.x;
  box.y_min = min_point.y;
  box.z_min = min_point.z;
  box.x_max = max_point.x;
  box.y_max = max_point.y;
  box.z_max = max_point.z;

  return box;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::LoadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::StreamPcd(
    std::string data_path) {
  // directory_iterator() constructs the end iterator
  // directory_iterator(directory p) constructs the first entry in the
  // directory p, if there is no such directory, returns the end iterator
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{data_path},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
