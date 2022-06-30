#ifndef LASER_MAPPING_HPP_
#define LASER_MAPPING_HPP_

#include <memory>

#include <array>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/sensor_data/cloud_data.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "aloam_velodyne/cloud_filter/voxel_filter.hpp"
#include "aloam_velodyne/aloam/sub_map.hpp"

#include "aloam_velodyne/aloam/optimizer.hpp"

namespace aloam {

class LaserMapping {
  public:
    // submap configuration
    static constexpr double kSubMapTileResolution = 50.0;

    static constexpr int kNumSubMapTilesX = 21;
    static constexpr int kNumSubMapTilesY = 21;
    static constexpr int kNumSubMapTilesZ = 11;
    static constexpr int kNumSubMapTiles = kNumSubMapTilesX * kNumSubMapTilesY * kNumSubMapTilesZ;

    static constexpr int kReanchorMargin = 3;
    static constexpr int kLocalMapRadius = 2;

    LaserMapping(void);

    bool Update(
      const CloudData::CloudT::Ptr sharp_points,
      const CloudData::CloudT::Ptr flat_points,
      const Eigen::Matrix4f& odom_scan_to_scan,
      Eigen::Matrix4f& lidar_odometry
    );

    Eigen::Vector3d GetRelativeTranslation() {
      return pose_.relative.t;
    }

    Eigen::Quaterniond GetRelativeRotation() {
      return pose_.relative.q;
    }

    void GetGlobalMap(CloudData::CloudT::Ptr &cloud_ptr);

    private:
    // matching config parameters
    struct {
      int min_num_sharp_points{10};
      int min_num_flat_points{50};

      double distance_thresh{1.0};

      int num_threads{4};
      int max_num_iteration{4};
      double max_solver_time{0.05};
      Optimizer::Config registration_config;
    } config_;

    // odometry
    struct Pose {
      Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
      Eigen::Vector3d t = Eigen::Vector3d::Zero();
    };

    struct {
      Pose scan_map_odometry;
      Pose relative;
      Pose scan_scan_odometry;
    } pose_;

    // filter
    struct {
      std::unique_ptr<VoxelFilter> sharp_filter_ptr_{nullptr};
      std::unique_ptr<VoxelFilter> flat_filter_ptr_{nullptr};
    } filter_;
    // target line & plane feature kdtree
    struct {
      pcl::KdTreeFLANN<CloudData::PointT>::Ptr sharp;
      pcl::KdTreeFLANN<CloudData::PointT>::Ptr flat;
    } kdtree_;
    // submap
    std::unique_ptr<SubMap> submap_ptr_{nullptr};

    bool InitParams(const YAML::Node& config_node);
    bool InitFilters(const YAML::Node& config_node);
    bool InitKdTrees(void);
    bool InitSubMap(const YAML::Node& config_node);

    bool HasSufficientFeaturePoints(const SubMap::LocalMap &local_map);
    bool SetTargetPoints(SubMap::LocalMap& local_map);

    bool ProjectToMapFrame(
      const CloudData::CloudT::Ptr& source,
      CloudData::CloudT::Ptr& query
    );
    int AddEdgeFactors(
      const CloudData::CloudT::Ptr source,
      const CloudData::CloudT::Ptr target,
      Optimizer &aloam_registration 
    );
    int AddPlaneFactors(
      const CloudData::CloudT::Ptr source,
      const CloudData::CloudT::Ptr target,
      Optimizer &aloam_registration 
    );

    bool PredictScanMapOdometry(const Eigen::Matrix4f& odom_scan_to_scan);
    bool UpdateRelativePose(void);
    bool UpdateOdometry(Eigen::Matrix4f& lidar_odometry);
};

} // namespace aloam

#endif // LASER_MAPPING_HPP_