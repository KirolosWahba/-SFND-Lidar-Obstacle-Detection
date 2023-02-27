// Author: Kirolos Wahba Moheeb
// February 20, 2023

#ifndef KDTREE_H_
#define KDTREE_H_

#include "render/render.h"
#include <chrono>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT> *&currentNode, unsigned int depth, Node<PointT> *&insertedNode)
	{
		if(currentNode == nullptr)
		{
			currentNode = insertedNode;
		}else
		{
			uint currentDimension = depth%3;
			if(currentDimension == 0)
			{
				if(insertedNode->point.x < currentNode->point.x)
				{
					insertHelper(currentNode->left, depth+1, insertedNode);
				}else
				{
					insertHelper(currentNode->right, depth+1, insertedNode);
				}
			}else if(currentDimension == 1)
			{
				if(insertedNode->point.y < currentNode->point.y)
				{
					insertHelper(currentNode->left, depth+1, insertedNode);
				}else
				{
					insertHelper(currentNode->right, depth+1, insertedNode);
				}
			}else
			{
				if(insertedNode->point.z < currentNode->point.z)
				{
					insertHelper(currentNode->left, depth+1, insertedNode);
				}else
				{
					insertHelper(currentNode->right, depth+1, insertedNode);
				}
			}
		}
	}

	void insert(PointT point, int id)
	{
		Node<PointT>* insertedNode = new Node<PointT>(point, id);
		insertHelper(root, 0, insertedNode);
	}

	void searchHelper(Node<PointT> *&currentNode, unsigned int depth, PointT target, float distanceTol, std::vector<int> &ids)
	{
		if(currentNode != nullptr)
		{
			
			std::vector<float> windowMax, windowMin;
			windowMax.push_back(target.x + distanceTol);
			windowMax.push_back(target.y + distanceTol);
			windowMax.push_back(target.z + distanceTol);
			windowMin.push_back(target.x - distanceTol);
			windowMin.push_back(target.y - distanceTol);
			windowMin.push_back(target.z - distanceTol);

			if((currentNode->point.x<=windowMax[0] && currentNode->point.x>=windowMin[0]) &&
			   (currentNode->point.y<=windowMax[1] && currentNode->point.y>=windowMin[1]) &&
			   (currentNode->point.z<=windowMax[2] && currentNode->point.z>=windowMin[2]))
			{
				float xDistance = fabs(target.x-currentNode->point.x);
				float yDistance = fabs(target.y-currentNode->point.y);
				float zDistance = fabs(target.z-currentNode->point.z);
				if(sqrtf(xDistance*xDistance+yDistance*yDistance+zDistance*zDistance) <= distanceTol){
					ids.push_back(currentNode->id);
				}
			}

			uint currentDimension = depth%3;
			if(currentDimension == 0)
			{
				if(windowMin[currentDimension]<currentNode->point.x)
				{
					searchHelper(currentNode->left, depth+1, target, distanceTol, ids);
				}
				if(windowMax[currentDimension]>currentNode->point.x)
				{
					searchHelper(currentNode->right, depth+1, target, distanceTol, ids);
				}	
			}else if(currentDimension == 1)
			{
				if(windowMin[currentDimension]<currentNode->point.y)
				{
					searchHelper(currentNode->left, depth+1, target, distanceTol, ids);
				}
				if(windowMax[currentDimension]>currentNode->point.y)
				{
					searchHelper(currentNode->right, depth+1, target, distanceTol, ids);
				}	
			}else
			{
				if(windowMin[currentDimension]<currentNode->point.z)
				{
					searchHelper(currentNode->left, depth+1, target, distanceTol, ids);
				}
				if(windowMax[currentDimension]>currentNode->point.z)
				{
					searchHelper(currentNode->right, depth+1, target, distanceTol, ids);
				}	
			}
		}
	}

	std::vector<int> search(PointT target, float distanceTol)
	{
		// auto startTime = std::chrono::steady_clock::now();
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		// auto endTime = std::chrono::steady_clock::now();
		// auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  		// std::cout << " Searching took " << elapsedTime.count() << " milliseconds" << std::endl;

		return ids;
	}
	

};
#endif /* KDTREE_H_ */
