#include "aloam_velodyne/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace aloam {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    LOG(INFO) << "Voxel Filter params:" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}

bool VoxelFilter::Filter(const CloudData::CloudT::Ptr& input_cloud_ptr, CloudData::CloudT::Ptr& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} 