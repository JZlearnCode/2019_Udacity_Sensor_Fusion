/* \author Udacity */
// Quiz on implementing kd tree

#include <math.h>
#include "../../render/render.h"

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
  // insertHelper function since passing a pointer to a function is the same as
  // passing by value, (not passing by reference)
  void insertHelper(Node** node, uint depth, std::vector<float> point, int id) {
    // first inserted point becomes root of the tree
    if (*node == NULL) {
      //*node to get Node* nodePtr
      *node = new Node(point, id);
    } else {
      uint current_dimension = depth % 2;
      // Depth even: X split
      // left if less than x, or right if greater
      if (point[current_dimension] < ((*node)->point[current_dimension])) {
        insertHelper(&((*node)->left), depth + 1, point, id);
      }
      // Depth out: y slipt
      // left if less than y, right if greater
      else {
        insertHelper(&((*node)->right), depth + 1, point, id);
      }
    }
  }
  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly within the root
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(std::vector<float> target, Node* node, int depth,
                    float distanceTol, std::vector<int>& ids) {
    if (node != NULL) {
      // 1. If the current node point is within this box, directly calculate the
      // distance
      //    and see if the point id should be added to the list of nearby ids
      if ((fabs(node->point[0] - target[0]) <= distanceTol) &&
          (fabs(node->point[1] - target[1]) <= distanceTol)) {
        float dx = node->point[0] - target[0];
        float dy = node->point[1] - target[1];
        float distance = sqrt(pow(dx, 2) + pow(dy, 2));
        if (distance <= distanceTol) {
          ids.push_back(node->id);
        }
      }
      // 2. If the left boundary of box is to the left of dividing line,
      // potential neighbor exists on the left of dividing line
      //    If the right boundary of box is to the right of dividing line,
      //    potential neighbor exists on the right of dividing line even depth,
      //    split by x, odd deptth, split by y
      if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
        searchHelper(target, node->left, depth + 1, distanceTol, ids);
      }
      if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
        searchHelper(target, node->right, depth + 1, distanceTol, ids);
      }
    }
  }

  // return a list of point ids in the tree that are within distance of target
  // With KD tree, compare distance within a boxed square that is 2x distanceTol
  // for length, centered around the target point.
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);
    return ids;
  }
};
