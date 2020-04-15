/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


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
template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void inserHelper(Node** node, uint depth, PointT point, int id)
	{
		//Tree is empty
		if(*node == NULL)
		{
			std::vector<float> vectorPoint(point.data, point.data + 3);
			*node = new Node(vectorPoint, id);

		}
		else
		{
			//Calculate current dimention
			uint cd = depth % 3;

			if(point.data[cd] < ((*node)->point[cd]))
			{
				inserHelper(&((*node)->left),depth+1, point, id);
			}
			else
			{
				inserHelper(&((*node)->right),depth+1, point, id);
			}
			
		}
		
	}


	void insertCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		for(uint id=0; id < cloud->points.size(); id++)
		{
			inserHelper(&root,0, cloud->points[id], id);
		}
	}

	void searchHelper(PointT target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if((node->point[0] >= (target.data[0]-distanceTol) && node->point[0] <= (target.data[0]+distanceTol)) && (node->point[1] >= (target.data[1]-distanceTol) && node->point[1] <= (target.data[1]+distanceTol)) && (node->point[2] >= (target.data[2]-distanceTol) && node->point[2] <= (target.data[2]+distanceTol)))
			{
			float distance = sqrt((node->point[0] - target.data[0])*(node->point[0] - target.data[0]) + (node->point[1] - target.data[1])*(node->point[1] - target.data[1]) + (node->point[2] - target.data[2])*(node->point[2] - target.data[2]));
			if(distance <= distanceTol)
				ids.push_back(node->id);

			}

			//check across boundary
			if((target.data[depth%3] - distanceTol) < node->point[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
		
			if((target.data[depth%3] + distanceTol) > node->point[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root,0, distanceTol, ids);
		return ids;
	}
};




