#ifndef LASER_ODOMETRY_FLOW_HPP_
#define LASER_ODOMETRY_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/subscriber/cloud_subscriber.hpp"
#include "aloam_velodyne/subscriber/odometry_subscriber.hpp"

#include "aloam_velodyne/publisher/cloud_publisher.hpp"
#include "aloam_velodyne/publisher/odometry_publisher.hpp"
#include "aloam_velodyne/publisher/path_publisher.hpp"

#include "aloam_velodyne/aloam/laser_odometry.hpp"

namespace aloam {

class LaserOdometryFlow {
  public:
    LaserOdometryFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool ReadData(void);
    bool HasData(void);
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    // config parameters
    struct {
      int num_frames_skip{1};
    } config_;

    // whether the front end is inited
    bool inited_{false};

    // inputs: pointclouds and features
    std::unique_ptr<CloudSubscriber<CloudData>> filtered_cloud_sub_ptr_{nullptr};
    std::deque<CloudData> filtered_cloud_buff_;
    CloudData filtered_cloud_;

    std::unique_ptr<CloudSubscriber<CloudData>> corner_points_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_sharp_buff_;
    CloudData corner_points_sharp_;

    std::unique_ptr<CloudSubscriber<CloudData>> corner_points_less_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_less_sharp_buff_;
    CloudData corner_points_less_sharp_;

    std::unique_ptr<CloudSubscriber<CloudData>> surf_points_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_flat_buff_;
    CloudData surf_points_flat_;

    std::unique_ptr<CloudSubscriber<CloudData>> surf_points_less_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_less_flat_buff_;
    CloudData surf_points_less_flat_;

    // Laser Odometry
    std::unique_ptr<LaserOdometry> laser_odometry_ptr_{nullptr};

    // outputs: pointclouds, features, and odometry poses
    std::unique_ptr<CloudPublisher> mapping_full_points_pub_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> mapping_sharp_points_pub_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> mapping_flat_points_pub_ptr_{nullptr};

    std::unique_ptr<OdometryPublisher> odom_laser_odometry_pub_ptr_;
    Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();

    // std::unique_ptr<PathPublisher> path_laser_odometry_pub_ptr_{nullptr};
};

} // namespace aloam

#endif // LASER_ODOMETRY_FLOW_HPP_