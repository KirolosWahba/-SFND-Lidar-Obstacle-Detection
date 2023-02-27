// Author: Kirolos Wahba Moheeb
// February 20, 2023

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "./kdtree.h"

template<typename PointT>
void proximity(int pointID, typename pcl::PointCloud<PointT>::Ptr cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, std::vector<bool>& processedPoints, float distanceTol);

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize);

#endif /* CLUSTER_H_ */