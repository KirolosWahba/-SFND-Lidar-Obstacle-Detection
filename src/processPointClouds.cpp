// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr croppedCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr roofFreeCroppedCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);

    // Cropping the scene to the interested area only
    pcl::CropBox<PointT> cropBoxObject;
    cropBoxObject.setInputCloud(filteredCloud);
    cropBoxObject.setMin(minPoint);
    cropBoxObject.setMax(maxPoint);
    cropBoxObject.filter(*croppedCloud);

    // Removing the noisy data from the car's roof
    cropBoxObject.setInputCloud(croppedCloud); 
    cropBoxObject.setMin(Eigen::Vector4f(-1.5, -1.5, -1, 1));
    cropBoxObject.setMax(Eigen::Vector4f(2.7, 1.5, 0, 1));
    cropBoxObject.setNegative(true);
    cropBoxObject.filter(*roofFreeCroppedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roofFreeCroppedCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>), planeCloud (new pcl::PointCloud<PointT>);
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);
    // cloud.swap (planeCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


/*
// Segment plane function using PCL library
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    
    // Segment the largest planar component from the cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
      
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}*/

// Segment plane function custom implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time Ransac process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
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
		PointT firstPoint = cloud->points.at(*indexItr);
		indexItr++;
		PointT secondPoint = cloud->points.at(*indexItr);
		indexItr++;
		PointT thirdPoint = cloud->points.at(*indexItr);

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
			if(distance<=distanceThreshold)
				inliersLocal.insert(index);
		}

		if(inliersLocal.size() > inliersResult.size()){
			inliersResult = inliersLocal;
		}
	}
    for(auto inlier:inliersResult)
        inliers->indices.emplace_back(inlier);
        
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


// Clustering function custom implementation
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>;
 
    for (int i=0; i<cloud->size(); i++) 
    	tree->insert((*cloud)[i],i); 

  	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/*
// Clustering function using PCL library
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.emplace_back(cloud_cluster);
        j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}