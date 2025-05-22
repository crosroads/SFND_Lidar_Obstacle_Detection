/* RANSAC 3D Implementation */
#ifndef RANSAC3D_H
#define RANSAC3D_H

#include <pcl/common/common.h>
#include <unordered_set>
#include <ctime>
#include <cstdlib>
#include <cmath>

// RANSAC plane segmentation function
template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // For max iterations 
    for(int i = 0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit plane
        std::unordered_set<int> inliers;
        while(inliers.size() < 3) inliers.insert(rand()%(cloud->points.size()));

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Calculate plane coefficients from 3 points
        // Plane equation: ax + by + cz + d = 0
        float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        float d = -(a*x1 + b*y1 + c*z1);

        // Measure distance between every point and fitted plane
        for (int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0)
                continue;
            
            float x4 = cloud->points[index].x;
            float y4 = cloud->points[index].y;
            float z4 = cloud->points[index].z;
            
            // Distance from point to plane
            float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);
            
            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceThreshold)
                inliers.insert(index);
        }

        // Keep the largest set of inliers
        if(inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    
    return inliersResult;
}

// RANSAC line segmentation function (2D)
template<typename PointT>
std::unordered_set<int> RansacLine(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // For max iterations 
    for(int i = 0; i < maxIterations; i++)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while(inliers.size() < 2) inliers.insert(rand()%(cloud->points.size()));

        float x1, y1, x2, y2;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // Line equation: ax + by + c = 0
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1*y2 - x2*y1;
        
        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0)
                continue;
            
            float x3 = cloud->points[index].x;
            float y3 = cloud->points[index].y;
            
            // Distance from point to line
            float dist = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);
            
            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceThreshold)
                inliers.insert(index);
        }

        // Keep the largest set of inliers
        if(inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    
    return inliersResult;
}

#endif /* RANSAC3D_H */