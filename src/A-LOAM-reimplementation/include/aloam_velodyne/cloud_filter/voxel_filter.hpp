#ifndef VOXEL_FILTER_HPP_
#define VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>
#include "aloam_velodyne/sensor_data/cloud_data.hpp"

namespace aloam {

class VoxelFilter {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CloudT::Ptr& input_cloud_ptr, CloudData::CloudT::Ptr& filtered_cloud_ptr);

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::PointT> voxel_filter_;
};

}

#endif // VOXEL_FILTER_HPP_