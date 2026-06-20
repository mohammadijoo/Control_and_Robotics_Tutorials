#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

int main(int argc, char** argv)
{
    using PointT = pcl::PointXYZ;

    pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>("model_cloud.pcd", *source) < 0 ||
        pcl::io::loadPCDFile<PointT>("scene_cloud.pcd", *target) < 0)
    {
        std::cerr << "Error loading clouds" << std::endl;
        return -1;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);

    icp.setMaxCorrespondenceDistance(0.02);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<PointT> aligned;
    icp.align(aligned);

    std::cout << "ICP has converged: " << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;

    Eigen::Matrix4f T = icp.getFinalTransformation();
    std::cout << "Estimated transform:\n" << T << std::endl;

    return 0;
}
      
