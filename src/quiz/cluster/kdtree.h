/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node *&currentNode, unsigned int depth, Node *&insertedNode)
	{
		if(currentNode == nullptr)
		{
			currentNode = insertedNode;
		}else
		{
			uint currentDimension = depth%2;
			if(insertedNode->point[currentDimension]<currentNode->point[currentDimension])
			{
				insertHelper(currentNode->left, depth+1, insertedNode);
			}else
			{
				insertHelper(currentNode->right, depth+1, insertedNode);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Node* insertedNode = new Node(point, id);
		insertHelper(root, 0, insertedNode);
	}

	void searchHelper(Node *&currentNode, unsigned int depth, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		if(currentNode != nullptr)
		{
			
			std::vector<float> windowMax, windowMin;
			windowMax.push_back(target[0] + distanceTol);
			windowMax.push_back(target[1] + distanceTol);
			windowMin.push_back(target[0] - distanceTol);
			windowMin.push_back(target[1] - distanceTol);

			if((currentNode->point[0]<=windowMax[0] && currentNode->point[0]>=windowMin[0]) && (currentNode->point[1]<=windowMax[1] && currentNode->point[1]>=windowMin[1]))
			{
				float xDistance = fabs(target[0]-currentNode->point[0]);
				float yDistance = fabs(target[1]-currentNode->point[1]);
				
				if(sqrtf(xDistance*xDistance+yDistance*yDistance) <= distanceTol){
					ids.push_back(currentNode->id);
				}
			}

			uint currentDimension = depth%2;
			if(windowMin[currentDimension]<currentNode->point[currentDimension])
			{
				// insertHelper(currentNode->left, depth+1, insertedNode);
				searchHelper(currentNode->left, depth+1, target, distanceTol, ids);
			}
			if(windowMax[currentDimension]>currentNode->point[currentDimension])
			{
				// insertHelper(currentNode->right, depth+1, insertedNode);
				searchHelper(currentNode->right, depth+1, target, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		auto startTime = std::chrono::steady_clock::now();
		std::vector<int> ids;

		searchHelper(root, 0, target, distanceTol, ids);

		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  		std::cout << " Searching took " << elapsedTime.count() << " milliseconds" << std::endl;

		return ids;
	}
	

};




