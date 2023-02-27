/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time Ransac process
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(int i=0; i<=maxIterations; i++){
		std::unordered_set<int> inliersLocal;
		// Randomly sample subset and fit line
		pcl::PointXYZ firstPoint = cloud->points.at(rand() % cloud->width);
		pcl::PointXYZ secondPoint = cloud->points.at(rand() % cloud->width);

		// Line equation Ax + By + C = 0
		// Line equation (y1 - y2)x + (x2 - x1)y + (x1*y2 - x2*y1) = 0
		float a = (firstPoint.y - secondPoint.y);
		float b = (secondPoint.x - firstPoint.x);
		float c = (firstPoint.x*secondPoint.y - secondPoint.x*firstPoint.y);
		// Measure distance between every point and fitted line
		// for(auto point:cloud->points){
			
		// }
		for (int index = 0; index != cloud->points.size(); index++) {
			//distance =|Ax+By+C|/sqrt( A^2 + B^2 )
			float pointX = cloud->points.at(index).x;
			float pointY = cloud->points.at(index).y;
			float distance = fabs(a*pointX + b*pointY + c) / sqrt(a*a+b*b);
			// If distance is smaller than threshold count it as inlier
			if(distance<=distanceTol)
				inliersLocal.insert(index);
		}

		if(inliersLocal.size() > inliersResult.size()){
			inliersResult = inliersLocal;
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time Ransac process
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(int i=0; i<=maxIterations; i++){
		std::unordered_set<int> inliersLocal;
		// Randomly sample subset and fit plane
		while(inliersLocal.size()<3){
			inliersLocal.insert(rand() % cloud->points.size());
		}
		auto indexItr = inliersLocal.begin();
		pcl::PointXYZ firstPoint = cloud->points.at(*indexItr);
		indexItr++;
		pcl::PointXYZ secondPoint = cloud->points.at(*indexItr);
		indexItr++;
		pcl::PointXYZ thirdPoint = cloud->points.at(*indexItr);

		// Two vectors laying in the plane
		pcl::PointXYZ vector1(secondPoint.x-firstPoint.x, secondPoint.y-firstPoint.y, secondPoint.z-firstPoint.z);
		pcl::PointXYZ vector2(thirdPoint.x-firstPoint.x, thirdPoint.y-firstPoint.y, thirdPoint.z-firstPoint.z);

		pcl::PointXYZ normalVector(vector1.y*vector2.z-vector1.z*vector2.y,
								   -vector1.x*vector2.z+vector1.z*vector2.x,
								   vector1.x*vector2.y-vector1.y*vector2.x);
		
		// Plane equation Ax + By + Cz + D = 0
		float a = normalVector.x;
		float b = normalVector.y;
		float c = normalVector.z;
		float D = -( a*firstPoint.x + b*firstPoint.y + c*firstPoint.z);
		
		// Measure distance between every point and fitted line
		for (int index = 0; index != cloud->points.size(); index++) {
			if(inliersLocal.count(index))
				continue;
			
			//distance =|Ax + By + Cz + D|/sqrt( A^2 + B^2 + c^2 )
			float pointX = cloud->points.at(index).x;
			float pointY = cloud->points.at(index).y;
			float pointZ = cloud->points.at(index).z;
			float distance = fabs(a*pointX + b*pointY + c*pointZ + D) / sqrt(a*a + b*b + c*c);
			// If distance is smaller than threshold count it as inlier
			if(distance<=distanceTol)
				inliersLocal.insert(index);
		}

		if(inliersLocal.size() > inliersResult.size()){
			inliersResult = inliersLocal;
		}
	}
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
