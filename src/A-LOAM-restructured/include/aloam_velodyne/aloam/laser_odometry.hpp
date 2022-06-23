#ifndef LASER_ODOMETRY_HPP_
#define LASER_ODOMETRY_HPP_

#include <memory>

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/sensor_data/cloud_data.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "aloam_velodyne/aloam/optimizer.hpp"

namespace aloam {

class LaserOdometry {
  public:
    LaserOdometry(void);

    bool Update(
      CloudData::CloudT::Ptr corner_sharp,
      CloudData::CloudT::Ptr corner_less_sharp,
      CloudData::CloudT::Ptr surf_flat,
      CloudData::CloudT::Ptr surf_less_flat,
      Eigen::Matrix4f& lidar_odometry
    );

  private:
    // Corner feature correspondences
    struct CornerPointAssociation {
      // index of point in current frame
      int query_index{-1};

      double ratio{1.0};
      
      // indices of associated points in last frame
      int associated_x_index{-1};
      int associated_y_index{-1};

      inline bool IsValid(void) const {
        return (query_index >= 0) && (associated_x_index >= 0) && (associated_y_index >= 0);
      }
    };

    // Plane feature correspondences
    struct SurfacePointAssociation {
      int query_index{-1};

      double ratio{1.0};
      
      int associated_x_index{-1};
      int associated_y_index{-1};
      int associated_z_index{-1};

      inline bool IsValid(void) const {
        return (query_index >= 0) && (associated_x_index >= 0) && (associated_y_index >= 0) && (associated_z_index >= 0);
      }
    };

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitKdTrees(void);

    bool TransformToStart(const CloudData::PointT &input, CloudData::PointT &output);

    bool AssociateCornerPoints(
      const CloudData::CloudT &corner_sharp,
      std::vector<CornerPointAssociation> &corner_point_associations
    );
    bool AssociateSurfacePoints(
      const CloudData::CloudT &surf_flat,
      std::vector<SurfacePointAssociation> &surface_point_associations
    );

    int AddEdgeFactors(
        const CloudData::CloudT &corner_sharp,
        const std::vector<CornerPointAssociation> &corner_point_associations,
        Optimizer &aloam_registration
    );
    int AddPlaneFactors(
        const CloudData::CloudT &surf_flat,
        const std::vector<SurfacePointAssociation> &surface_point_associations,
        Optimizer &aloam_registration
    );

    bool SetTargetPoints(
      const CloudData::CloudT::Ptr &corner_less_sharp,
      const CloudData::CloudT::Ptr &surf_less_flat
    );

    bool UpdateOdometry(Eigen::Matrix4f& lidar_odometry);

  private:
    // config parameters
    struct {
      double scan_period{0.10};
      
      double distance_thresh{25.0};
      double scan_thresh{2.50};

      int num_threads{4};
      int max_num_iteration{4};
      double max_solver_time{0.05};
      Optimizer::Config registration_config;
    } config_;

    // kdtree for target corner & plane feature points
    struct {
      CloudData::CloudT::Ptr candidate_corner_ptr;
      pcl::KdTreeFLANN<CloudData::PointT>::Ptr corner;

      CloudData::CloudT::Ptr candidate_surface_ptr;
      pcl::KdTreeFLANN<CloudData::PointT>::Ptr surface;
    } kdtree_;

    // whether the front end is inited
    bool inited_{false};

    // relative pose
    Eigen::Quaterniond dq_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d dt_ = Eigen::Vector3d::Zero();

    // odometry pose
    Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_ = Eigen::Vector3d::Zero();
};

} // namespace aloam

#endif // LASER_ODOMETRY_HPP_