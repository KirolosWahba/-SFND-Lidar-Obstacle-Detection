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
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "PCL", Color(1,1,1));
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* PointCloudProcesser = new ProcessPointClouds<pcl::PointXYZ>();
    int maxIterations = 10;
    float distanceThreshold = 0.2;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr>  segmentedClouds= PointCloudProcesser->SegmentPlane(inputCloud, maxIterations, distanceThreshold);
    // renderPointCloud(viewer, segmentedClouds.first, "obstcleCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentedClouds.second, "roadCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PointCloudProcesser->Clustering(segmentedClouds.first, 1.0, 3, 30);

    // Clustering the obstacles point cloud
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        PointCloudProcesser->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = PointCloudProcesser->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Filtering the cloud
    Eigen::Vector4f windowMinPoint(-10, -5, -2, 1);
    Eigen::Vector4f windowMaxPoint(30, 7, 0.7, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.4, windowMinPoint, windowMaxPoint);

    int maxIterations = 10;
    float distanceThreshold = 0.2;
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr>  segmentedClouds = pointProcessorI->SegmentPlane(filteredCloud, maxIterations, distanceThreshold);
    // renderPointCloud(viewer, segmentedClouds.first, "obstcleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentedClouds.second, "roadCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedClouds.first, 0.5, 5, 300);

    // Clustering the obstacles point cloud
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

    // For tuning only
    // Box windowBox;
    // windowBox.x_min = -10;
    // windowBox.y_min = -5;
    // windowBox.z_min = -2;
    // windowBox.x_max = 30;
    // windowBox.y_max = 8;
    // windowBox.z_max = 0.7;

    // Box roofBox;
    // roofBox.x_min = -2;
    // roofBox.y_min = -1.5;
    // roofBox.z_min = -1;
    // roofBox.x_max = 2.8;
    // roofBox.y_max = 1.5;
    // roofBox.z_max = 0;
    // renderBox(viewer, windowBox, 15);
    // renderBox(viewer, roofBox, 16);
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // renderPointCloud(viewer,filteredCloud,"filteredCloud");
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator==stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();

        // Clear the viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    } 
}