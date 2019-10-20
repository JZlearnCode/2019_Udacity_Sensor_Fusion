/*
Implement Euclidean Clustering
Author : Jin Zhu
*/
#include <chrono>
#include <string>
#include <vector>
#include "kdtree.h"

void GrowCluster(int index, const std::vector<std::vector<float>> points,
                 std::vector<int>& cluster, std::vector<bool>& processed,
                 KdTree* tree, float distance_tolerance) {
  processed[index] = true;
  cluster.push_back(index);

  std::vector<int> nearby_neighbors =
      tree->Search(points[index], distance_tolerance);
  for (int id : nearby_neighbors) {
    if (!processed[id]) {
      GrowCluster(id, points, cluster, processed, tree, distance_tolerance);
    }
  }
}

std::vector<std::vector<int>> EuclideanCluster(
    const std::vector<std::vector<float>>& points, KdTree* tree,
    float distance_tolerance) {
  std::vector<std::vector<int>> clusters;
  // keep track of which point has been processed or not
  std::vector<bool> processed(points.size(), false);

  int i = 0;
  for (int i = 0; i < points.size(); i++) {
    if (!processed[i]) {
      std::vector<int> cluster;
      GrowCluster(i, points, cluster, processed, tree, distance_tolerance);
      clusters.push_back(cluster);
    }
  }
  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minClusterSize, int maxClusterSize) {
  KdTree* tree = new KdTree;
  int num_points = cloud->points.size() for (int i = 0; i <)
}
