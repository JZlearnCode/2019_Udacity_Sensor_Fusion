/* \author Udacity */
// Quiz on implementing simple RANSAC line fitting
// The point cloud is actually pcl::PointXYZ but the z component will be set to
// zero to make things easy to visualize in 2D space

#include <unordered_set>
#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
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
    int i = 0;
    for (auto itr = inliers.begin(); itr != inliers.end(); itr++) {
      std::cout << "i" << i << std::endl;
      x.push_back(cloud->points[*itr].x);
      y.push_back(cloud->points[*itr].y);
      z.push_back(cloud->points[*itr].z);
      i++;
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
      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;
      float z3 = point.z;
      // fabs: float abs
      float dist =
          fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);
      if (dist <= distanceTol) inliers.insert(index);
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

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 3D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
