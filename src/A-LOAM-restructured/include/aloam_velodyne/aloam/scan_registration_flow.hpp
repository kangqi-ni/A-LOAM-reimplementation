#ifndef SCAN_REGISTRATION_FLOW_HPP_
#define SCAN_REGISTRATION_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "aloam_velodyne/subscriber/tf_listener.hpp"
#include "aloam_velodyne/subscriber/cloud_subscriber.hpp"
#include "aloam_velodyne/publisher/cloud_publisher.hpp"

#include "aloam_velodyne/aloam/scan_registration.hpp"

namespace aloam {

class ScanRegistrationFlow {
  public:
    ScanRegistrationFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool InitCalibration(void);
    bool ReadData(void);
    bool HasData(void);
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    // inputs: pointclouds
    std::unique_ptr<CloudSubscriber<CloudData>> cloud_sub_ptr_{nullptr};
    std::deque<CloudData> cloud_data_buff_;
    CloudData current_cloud_data_;

    // scan registration
    std::unique_ptr<ScanRegistration> scan_registration_ptr{nullptr};
    std::unique_ptr<CloudPublisher> filtered_cloud_pub_ptr_{nullptr};
    CloudData::CloudT::Ptr filtered_cloud_data_;

    // outputs: registered scans
    std::unique_ptr<CloudPublisher> corner_points_sharp_pub_ptr_{nullptr};
    CloudData::CloudT::Ptr  corner_sharp_;
    std::unique_ptr<CloudPublisher> corner_points_less_sharp_pub_ptr_{nullptr};
    CloudData::CloudT::Ptr  corner_less_sharp_;
    std::unique_ptr<CloudPublisher> surf_points_flat_pub_ptr_{nullptr};
    CloudData::CloudT::Ptr  surf_flat_;
    std::unique_ptr<CloudPublisher> surf_points_less_flat_pub_ptr_{nullptr};
    CloudData::CloudT::Ptr  surf_less_flat_;
    std::unique_ptr<CloudPublisher> removed_points_pub_ptr_{nullptr};
};

} // namespace aloam

#endif // SCAN_REGISTRATION_FLOW_HPP_