// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

// filterRes defines resolution for voxel grid filtering
// minPoint, maxPoint defines region of interest
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  // Create the filtering object: downsample the datset using a leaf size of .2m
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
      new pcl::PointCloud<PointT>);

  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes,
                 filterRes);  // set resolution in x, y, z direction
  vg.filter(*cloudFiltered);

  // Region of interest filtering
  typename pcl::PointCloud<PointT>::Ptr cloudRegion(
      new pcl::PointCloud<PointT>);
  // CropBox(bool extract_removed_indices), set to true if you want to be able
  // to extarct the indices of points beling removed
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  // remove the points caused by lidar ray hitting root of the car
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 0));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 0));
  roof.setInputCloud(cloudRegion);
  // filter can return indices of points within the defined region (close to
  // roof of car)
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices) {
    inliers->indices.push_back(point);
  }
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    std::unordered_set<int>& inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points[index];
    if (inliers.count(index))
      planeCloud->points.push_back(point);
    else
      obstCloud->points.push_back(point);
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstCloud, planeCloud);
  return segResult;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  auto startTime = std::chrono::steady_clock::now();
  // TODO: Fill in this function
  // For max iterations
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

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    for (int index = 0; index < cloud->points.size(); index++) {
      // if the index is already in inlier (the 2 points that's originally
      // picked)
      if (inliers.count(index) > 0) continue;
      PointT point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float z3 = point.z;
      // fabs: float abs
      float d =
          fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);
      if (d <= distanceTol) inliers.insert(index);
    }

    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "Ransac took " << elapsedTime.count() << "milliseconds"
            << std::endl;
  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles create the KdTree object for the search method of the
  // extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  for (pcl::PointIndices getIndices : clusterIndices) {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(
        new pcl::PointCloud<PointT>);
    for (int idx : getIndices.indices) {
      cloudCluster->points.push_back(cloud->points[idx]);
    }
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    clusters.push_back(cloudCluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

// BoundingBox looks at the min and max point values of an input cloud and
// stores thoes parameters in a box struct container
template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  // directory_iterator() constructs the end iterator
  // directory_iterator(directory p) constructs the first entry in the directory
  // p, if there is no such directory, returns the end iterator
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}