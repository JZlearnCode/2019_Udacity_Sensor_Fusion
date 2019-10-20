/*
Implement Euclidean Clustering
Author : Jin Zhu
*/
// #include <chrono>
// #include <string>
// #include <vector>
// #include "kdtree.h"
// template <typename PointT>
// void GrowCluster(int index, typename pcl::PointCloud<PointT>::Ptr cloud,
//                  std::vector<int>& cluster_idx, std::vector<bool>& processed,
//                  KdTree* tree, float distance_tolerance) {
//   processed[index] = true;
//   cluster_idx.push_back(index);

//   std::vector<int> nearby_neighbors =
//       tree->Search(cloud->points[index], distance_tolerance);
//   for (int id : nearby_neighbors) {
//     if (!processed[id]) {
//       GrowCluster(id, cloud, cluster_idx, processed, tree,
//       distance_tolerance);
//     }
//   }
// }

// // std::vector<std::vector<int>> EuclideanCluster(
// //     const std::vector<std::vector<float>>& points, KdTree* tree,
// //     float distance_tolerance) {
// //   std::vector<std::vector<int>> clusters;
// //   // keep track of which point has been processed or not
// //   std::vector<bool> processed(points.size(), false);

// //   int i = 0;
// //   for (int i = 0; i < points.size(); i++) {
// //     if (!processed[i]) {
// //       std::vector<int> cluster;
// //       GrowCluster(i, points, cluster, processed, tree,
// distance_tolerance);
// //       clusters.push_back(cluster);
// //     }
// //   }
// //   return clusters;
// // }

// // todo: change from vector to not using vector, only use point cloud
// template <typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(
//     typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance,
//     int min_cluster_size, int max_cluster_size) {
//   std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
//   KdTree* tree = new KdTree;
//   int num_points = cloud->points.size();
//   for (int i = 0; i < num_points; i++) {
//     tree->InsertNode(cloud->points[i], i);
//   }

//   // keep track of which point has been processed or not
//   std::vector<bool> processed(num_points, false);
//   for (int i = 0; i < num_points; i++) {
//     if (!processed[i]) {
//       std::vector<int> cluster_idx;
//       typename pcl::PointCloud<PointT>::Ptr cluster;
//       GrowCluster(i, cloud, cluster_idx, processed, tree, cluster_tolerance);
//       if (cluster_idx.size() >= min_cluster_size &&
//           cluster_idx.size() <= max_cluster_size) {
//         // create point cloud cluster from vector of point indices
//         for (int j = 0; j < cluster_idx.size(); j++) {
//           cluster->points.push_back(cloud->points[cluster_idx[j]]);
//         }
//         // save the cluster
//         clusters.push_back(cluster);
//       }
//     }
//   }
//   return clusters;
// }
