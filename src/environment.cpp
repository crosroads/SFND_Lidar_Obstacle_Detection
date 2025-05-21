/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar.scan();
    renderPointCloud(viewer, cloud, "lidarCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(cloud, 1000, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.5, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // Filter the point cloud to focus on the region of interest and reduce data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = pointProcessorI->FilterCloud(
      inputCloud, 0.2,
      Eigen::Vector4f (-10, -5, -2, 1),
      Eigen::Vector4f (30, 8, 1, 1)
  );

  // Add a box to show where ego car roof points were removed
  Box roofBox;
  roofBox.x_min = -1.5;
  roofBox.y_min = -1.7;
  roofBox.z_min = -1.0;
  roofBox.x_max = 2.6;
  roofBox.y_max = 1.7;
  roofBox.z_max = -0.4;
  renderBox(viewer, roofBox, 999, Color(1,1,1), 0.5); // Using ID 999 to avoid conflicts

  // First render the filtered cloud as the base layer
  renderPointCloud(viewer, cloud_filtered, "cloud_filtered", Color(0.8, 0.8, 0.8));

  // Step 1. Segment the filtered cloud into two parts, road and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
      pointProcessorI->SegmentPlane(cloud_filtered, 100, 0.2);

  // Render the road plane with a green color
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  // Step 2. Cluster the obstacle cloud with appropriate parameters
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);

  // Step 3. Find bounding boxes for the clusters and render them
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);

      // Render each cluster with a distinct color
      renderPointCloud(
          viewer,
          cluster,
          "obstCloud"+std::to_string(clusterId),
          colors[clusterId % colors.size()]
      );

      // Create and render a bounding box around each cluster
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);

      ++clusterId;
  }
}




//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}
