/* \author Udacity */
// Functions and structs used to render the enviroment

#include "render.h"

void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                      std::string name, Color color) {
  if (color.r == -1) {
    // Select color based off of cloud intensity
    // PCLVisualizer uses ColorHandler objects to display dustom data
    // PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud,
    // const std::string &field_name) Uses an user given field to extract 1D
    // data and display the color at each point using a min-max lookup table.
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
  } else {
    // Select color based off input value
    viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b,
        name);
  }
  // setPointCloudRenderingProperties(int property, double valuee, const
  // std::string& id = 'cloud)
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color
void RenderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id,
               Color color, float opacity) {
  if (opacity > 1.0) opacity = 1.0;
  if (opacity < 0.0) opacity = 0.0;

  std::string cube = "box" + std::to_string(id);
  // viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length,
  // box.cube_width, box.cube_height, cube);
  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min,
                  box.z_max, color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, cube);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

  std::string cube_fill = "boxFill" + std::to_string(id);
  viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min,
                  box.z_max, color.r, color.g, color.b, cube_fill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_fill);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      color.r, color.g, color.b, cube_fill);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cube_fill);
}
