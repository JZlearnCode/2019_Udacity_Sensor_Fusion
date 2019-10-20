#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry>

struct Box {
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};
#endif