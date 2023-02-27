// Author: Kirolos Wahba Moheeb
// February 20, 2023

#include "cluster.h"

template<typename PointT>
void proximity(int pointID, typename pcl::PointCloud<PointT>::Ptr cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, std::vector<bool>& processedPoints, float distanceTol)
{
	processedPoints.at(pointID) = true;
	cluster->push_back(cloud->at(pointID));
	std::vector<int> nearbyPointsIDs = tree->search(cloud->at(pointID), distanceTol);
	for(int pointID:nearbyPointsIDs)
	{
		if (!processedPoints.at(pointID))
		{
			proximity(pointID, cluster, cloud, tree, processedPoints, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	
	std::vector<bool> processedPoints(cloud->size(), false);
	
	for (int pointID=0; pointID<cloud->size(); pointID++) 
	{
		if (processedPoints.at(pointID))
		{
			continue;
		}
		
		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);

		proximity(pointID, cluster, cloud, tree, processedPoints, distanceTol);

		if(cluster->size() < minSize || cluster->size() > maxSize)
		{
			continue; 
		}

		cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
		clusters.emplace_back(cluster);
	}
	
	return clusters;
}
