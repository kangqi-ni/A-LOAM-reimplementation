#ifndef CLOUD_DATA_HPP
#define CLOUD_DATA_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace aloam {

class CloudData {
public:
    using PointT = pcl::PointXYZI;
    using CloudT = pcl::PointCloud<PointT>;
    using CloudT_Ptr = CloudT::Ptr;
    
    CloudData():cloud_ptr(new CloudT()) {}

public:
    double time;
    CloudT_Ptr cloud_ptr;
};

}

#endif // CLOUD_DATA_HPP