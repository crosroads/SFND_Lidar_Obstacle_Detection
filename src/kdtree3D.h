/* 3D KD-Tree Implementation */
#ifndef KDTREE3D_H
#define KDTREE3D_H

#include <vector>
#include <queue>
#include <cmath>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :   point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct KdTree3D
{
    Node* root;

    KdTree3D()
    : root(NULL)
    {}

    ~KdTree3D()
    {
        delete root;
    }

    void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
    {
        if(*node == NULL)
            *node = new Node(point, id);
        else
        {
            // Calculate current dimension to split on (x, y, or z)
            uint cd = depth % 3;
            
            if(point[cd] < ((*node)->point[cd]))
                insertHelper(&((*node)->left), depth+1, point, id);
            else
                insertHelper(&((*node)->right), depth+1, point, id);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != NULL)
        {
            // Check if point is within the boundary box
            if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
                (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) &&
                (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)) )
            {
                float distance = sqrt(
                    (node->point[0] - target[0]) * (node->point[0] - target[0]) +
                    (node->point[1] - target[1]) * (node->point[1] - target[1]) +
                    (node->point[2] - target[2]) * (node->point[2] - target[2])
                );
                if(distance <= distanceTol)
                    ids.push_back(node->id);
            }
            
            // Check which side of the splitting plane to search
            uint cd = depth % 3;
            if(target[cd] - distanceTol < node->point[cd])
                searchHelper(target, node->left, depth+1, distanceTol, ids);
            if(target[cd] + distanceTol > node->point[cd])
                searchHelper(target, node->right, depth+1, distanceTol, ids);
        }
    }

    // Return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

// Helper function for Euclidean Clustering
template<typename PointT>
void clusterHelper(int index, const typename pcl::PointCloud<PointT>::Ptr cloud, 
                  std::vector<bool>& processed, KdTree3D* tree, 
                  float distanceTol, std::vector<int>& cluster)
{
    processed[index] = true;
    cluster.push_back(index);
    
    std::vector<float> point = {cloud->points[index].x, cloud->points[index].y, cloud->points[index].z};
    std::vector<int> nearby = tree->search(point, distanceTol);
    
    for(int id : nearby)
    {
        if(!processed[id])
            clusterHelper<PointT>(id, cloud, processed, tree, distanceTol, cluster);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster3D(
    const typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // Create KD-tree object
    KdTree3D* tree = new KdTree3D;
    
    // Insert points into KD-tree
    for(int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point, i);
    }
    
    // Track processed points
    std::vector<bool> processed(cloud->points.size(), false);
    
    // Find clusters
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(!processed[i])
        {
            std::vector<int> cluster_indices;
            clusterHelper<PointT>(i, cloud, processed, tree, clusterTolerance, cluster_indices);
            
            // Create point cloud for this cluster
            if(cluster_indices.size() >= minSize && cluster_indices.size() <= maxSize)
            {
                typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
                for(int idx : cluster_indices)
                    cloudCluster->points.push_back(cloud->points[idx]);
                
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                cloudCluster->is_dense = true;
                
                clusters.push_back(cloudCluster);
            }
        }
    }
    
    delete tree;
    return clusters;
}

#endif /* KDTREE3D_H */