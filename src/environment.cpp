/* \author Udacity */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr& viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

// real point cloud
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {
  inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3,
                                            Eigen::Vector4f(-10, -5, -2, 1),
                                            Eigen::Vector4f(30, 8, 1, 1));
  std::unordered_set<int> inliers =
      pointProcessorI->Ransac3D(inputCloud, 25, 3.0);

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmentCloud = pointProcessorI->SeparateClouds(inliers, inputCloud);
  // renderPointCloud(viewer, segmentaCloud.first, "obstCloud", Color(1, 0, 0));
  // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1,
  // 0));
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obsCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

// simulated point cloud
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  bool render_clusters = true;
  bool render_box = true;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  // use "new" keyword to put Lidar object on heap
  // since Lidar object could potentially be very large
  Lidar* lidar = new Lidar(cars, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
  // renderRays(viewer, lidar->position, inputCloud);
  renderPointCloud(viewer, inputCloud, "point_cloud", Color(0, 1, 0));
  std::cout << "lidar size" << sizeof(lidar) << std::endl;
  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  // separate plane and obstacles
  std::unordered_set<int> inliers =
      pointProcessor.Ransac3D(inputCloud, 100, 0.2);

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentCloud = pointProcessor.SeparateClouds(inliers, inputCloud);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

  // rendering
  // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1,
  // 0));

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size";
    pointProcessor.numPoints(cluster);
    if (render_clusters) {
      renderPointCloud(viewer, cluster, "obsCloud" + std::to_string(clusterId),
                       colors[clusterId % colors.size()]);
    }
    if (render_box) {
      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }
    ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
// XY: 45 degree angle view
// FPS: First Person Sense, gives the sensation of being in the car's driver
// seat viewer is passed in as a reference to a pointer, thus any changes done
// to viewr will prercist outside function
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char** argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
  ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
  // stream is a vector of chronologically ordered vector of all the file names
  std::vector<boost::filesystem::path> stream =
      pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear Viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
    cityBlock(viewer, &pointProcessorI, inputCloudI);

    streamIterator++;

    // loop back to the first file if finished
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }
    viewer->spinOnce();
  }
}