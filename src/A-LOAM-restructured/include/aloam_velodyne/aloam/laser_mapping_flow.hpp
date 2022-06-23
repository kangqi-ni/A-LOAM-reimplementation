#ifndef LASER_MAPPING_FLOW_HPP_
#define LASER_MAPPING_FLOW_HPP_

#include <memory>

#include <deque>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/sensor_data/cloud_data.hpp"
#include "aloam_velodyne/subscriber/cloud_subscriber.hpp"
#include "aloam_velodyne/subscriber/odometry_subscriber.hpp"

#include "aloam_velodyne/publisher/cloud_publisher.hpp"
#include "aloam_velodyne/publisher/odometry_publisher.hpp"
#include "aloam_velodyne/publisher/path_publisher.hpp"

#include "aloam_velodyne/publisher/tf_broadcaster.hpp"

#include "aloam_velodyne/aloam/laser_mapping.hpp"
#include "ros/publisher.h"

namespace aloam {

class LaserMappingFlow {
  public:
    LaserMappingFlow(ros::NodeHandle& nh);
    
    void Process();

    bool ReadData(void);

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool HasOdomData(void);
    bool HasCloudData(void);
    bool HasData(void);
    
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);

    void LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

  private:
    struct {
      int num_frames_skip{1};
    } config_;

    // whether the front end is inited
    bool inited_{false};

    // inputs: registered scans & laser odometry pose
    std::unique_ptr<CloudSubscriber<CloudData>> mapping_sharp_points_sub_ptr_{nullptr};
    std::deque<CloudData> mapping_sharp_points_buff_;
    CloudData mapping_sharp_points_;

    std::unique_ptr<CloudSubscriber<CloudData>> mapping_flat_points_sub_ptr_{nullptr};
    std::deque<CloudData> mapping_flat_points_buff_;
    CloudData mapping_flat_points_;

    std::unique_ptr<CloudSubscriber<CloudData>> mapping_full_sub_ptr_{nullptr};
    std::deque<CloudData> mapping_full_buff_;
    CloudData mapping_full_;

    std::shared_ptr<OdometrySubscriber> odom_laser_odometry_sub_ptr_;
    std::deque<PoseData> odom_laser_odometry_buff_;
    PoseData odom_laser_odometry_;

    ros::Subscriber odom_laser_odom_sub_ptr_;

    // A-LOAM laser mapping
    std::unique_ptr<LaserMapping> laser_mapping_ptr_{nullptr};

    // outputs
    std::unique_ptr<OdometryPublisher> odom_laser_mapping_pub_ptr_;
    Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();

    // std::unique_ptr<PathPublisher> path_laser_mapping_pub_ptr_;

    ros::Publisher odom_laser_mapping_high_frec_pub_ptr_;

    // std::unique_ptr<CloudPublisher> local_map_cloud_pub_ptr_;

    std::unique_ptr<CloudPublisher> global_map_cloud_pub_ptr_;
    std::unique_ptr<CloudPublisher> current_cloud_pub_ptr_;

    CloudData::CloudT::Ptr laser_cloud_surround_{new CloudData::CloudT()};
    CloudData::CloudT::Ptr laser_cloud_map_{new CloudData::CloudT()};

    std::thread process_thread_;

    std::mutex buf_;
};

} // namespace aloam

#endif // LASER_MAPPING_FLOW_HPP_