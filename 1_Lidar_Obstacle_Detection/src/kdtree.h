/*
Implementation of KD tree in 3D space for point cloud
Author : JZ
*/
#ifndef KDTREE_H_
#define KDTREE_H_
#include <math.h>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node* root;

  KdTree() : root(NULL) {}

  // if use Node* node as input parameter, root will still be nullptr outside
  // InsertNode function since passing a pointer to a function is the same as
  // passing by value, (not passing by reference)
  void InsertNodeHelper(Node** node, uint depth, std::vector<float> point,
                        int id) {
    // first inserted point becomes root of the tree
    if (*node == NULL) {
      //*node to get Node* nodePtr
      *node = new Node(point, id);
    } else {
      // current_dimension = 0: X split,  1: y split, 2: z split
      int current_dimension = depth % 3;
      // X split: left if less than x, or right if greater
      // Y split: left if less than y, or right if greater
      // Z split: left if less than z, or right if greater
      if (point[current_dimension] < ((*node)->point[current_dimension])) {
        InsertNodeHelper(&((*node)->left), depth + 1, point, id);
      } else {
        InsertNodeHelper(&((*node)->right), depth + 1, point, id);
      }
    }
  }
  void InsertNode(std::vector<float> point, int id) {
    // Insert a new point into the tree
    // the function should create a new node and place correctly within the root
    InsertNodeHelper(&root, 0, point, id);
  }

  void InsertNode(pcl::PointXYZI pcl_point, int id) {
    // Insert a new point into the tree
    // the function should create a new node and place correctly within the root
    std::vector<float> point = {pcl_point.x, pcl_point.y, pcl_point.z};
    InsertNode(point, id);
  }

  void SearchHelper(std::vector<float> target, Node* node, int depth,
                    float distance_tolerance, std::vector<int>& ids) {
    if (node != NULL) {
      // 1. If the current node point is within this box,
      // directly calculate the distance and see if the point id should be
      // added to the list of nearby ids
      float diff_x = fabs(node->point[0] - target[0]);
      float diff_y = fabs(node->point[1] - target[1]);
      float diff_z = fabs(node->point[2] - target[2]);
      if (diff_x <= distance_tolerance && diff_y <= distance_tolerance &&
          diff_z <= distance_tolerance) {
        float dx = node->point[0] - target[0];
        float dy = node->point[1] - target[1];
        float distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2));
        if (distance <= distance_tolerance) {
          ids.push_back(node->id);
        }
      }
      // 2. If the left boundary of box is to the left of dividing line,
      //    potential neighbor exists on the left of dividing line
      //    If the right boundary of box is to the right of dividing line,
      //    potential neighbor exists on the right of dividing line
      if ((target[depth % 3] - distance_tolerance) < node->point[depth % 3]) {
        SearchHelper(target, node->left, depth + 1, distance_tolerance, ids);
      }
      if ((target[depth % 3] + distance_tolerance) > node->point[depth % 3]) {
        SearchHelper(target, node->right, depth + 1, distance_tolerance, ids);
      }
    }
  }

  // return a list of point ids in the tree that are within distance of target
  // With KD tree, compare distance within a boxed square that is 2x distanceTol
  // for length, centered around the target point.
  std::vector<int> Search(std::vector<float> target, float distance_tolerance) {
    std::vector<int> ids;
    SearchHelper(target, root, 0, distance_tolerance, ids);
    return ids;
  }

  std::vector<int> Search(pcl::PointXYZI pcl_point, float distance_tolerance) {
    std::vector<float> target = {pcl_point.x, pcl_point.y, pcl_point.z};
    return Search(target, distance_tolerance);
  }
};
#endif