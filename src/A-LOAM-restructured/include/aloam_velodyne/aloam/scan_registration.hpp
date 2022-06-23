#ifndef SCAN_REGISTRATION_HPP_
#define SCAN_REGISTRATION_HPP_

#include <memory>

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/sensor_data/cloud_data.hpp"

#include <pcl/filters/voxel_grid.h>


namespace aloam {

enum FeaturePoint {
  CORNER_SHARP = 0,
  CORNER_LESS_SHARP = 1,
  SURF_FLAT = 2,
  SURF_LESS_FLAT = 3,
  NUM_TYPES = 4
};

class ScanRegistration {
  public:
    static constexpr int kScanIndexSize = 64;
    static constexpr int kPointIndexSize = 400000;

    ScanRegistration(void);

    bool Update(
      const CloudData& input_cloud, 
      CloudData::CloudT::Ptr &output_cloud,
      CloudData::CloudT::Ptr &corner_sharp,
      CloudData::CloudT::Ptr &corner_less_sharp,
      CloudData::CloudT::Ptr &surf_flat,
      CloudData::CloudT::Ptr &surf_less_flat
    );
  
  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitFilters(const YAML::Node& config_node);

    bool FilterByRange(const CloudData::CloudT &input_cloud, CloudData::CloudT &output_cloud);

    bool GetScanId(const float &angle, int &scan_id);
    float GetCurvature(const CloudData::CloudT &cloud, int point_index);

    bool SortPointCloudByScan(const CloudData::CloudT &input_cloud, CloudData::CloudT &output_cloud);

    bool PickInNeighborhood(const CloudData::CloudT &cloud, const int point_index, const float thresh);
    bool GetFeaturePoints(
      const CloudData::CloudT &cloud, 
      CloudData::CloudT::Ptr &corner_sharp,
      CloudData::CloudT::Ptr &corner_less_sharp,
      CloudData::CloudT::Ptr &surf_flat,
      CloudData::CloudT::Ptr &surf_less_flat
    );

  private:
    struct {
      float scan_period;
      int num_scans;
      float min_range;
      int neighborhood_size;
      int num_sectors;
    } config_;

    struct {
      struct {
        int start[kScanIndexSize];
        int end[kScanIndexSize];
      } scan;

      struct {
        float curvature[kPointIndexSize];
        int index[kPointIndexSize];
        int picked[kPointIndexSize];
        int label[kPointIndexSize];
      } point;
    } index_;

    std::unique_ptr<pcl::VoxelGrid<CloudData::PointT>> surf_less_flat_filter_ptr_{nullptr};
};

} // namespace aloam

#endif // SCAN_REGISTRATION_HPP_