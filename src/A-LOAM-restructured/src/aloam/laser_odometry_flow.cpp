#include "aloam_velodyne/aloam/laser_odometry_flow.hpp"

#include "aloam_velodyne/aloam/laser_odometry.hpp"
#include "aloam_velodyne/publisher/path_publisher.hpp"
#include "glog/logging.h"

#include "aloam_velodyne/util/file_manager.hpp"
#include "aloam_velodyne/global_definition/global_definition.h"

namespace aloam {

LaserOdometryFlow::LaserOdometryFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/aloam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // Init params
    InitParam(config_node["laser_odometry"]["param"]);

    // Init subscribers to registered scans
    InitSubscribers(nh, config_node["scan_registration"]["publisher"]);

    // Front end odometry
    laser_odometry_ptr_ = std::make_unique<LaserOdometry>();

    // Init publishers
    InitPublishers(nh, config_node["laser_odometry"]["publisher"]);
}

bool LaserOdometryFlow::InitParam(const YAML::Node& config_node) {
    config_.num_frames_skip = config_node["num_frames_skip"].as<int>();

    return true;
}

bool LaserOdometryFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    filtered_cloud_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["queue_size"].as<int>()
    );

    corner_points_sharp_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["queue_size"].as<int>()
    );

    corner_points_less_sharp_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["queue_size"].as<int>()
    );

    surf_points_flat_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["queue_size"].as<int>()
    );

    surf_points_less_flat_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["queue_size"].as<int>()
    );

    return true;
}

bool LaserOdometryFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    mapping_full_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["full"]["topic_name"].as<std::string>(),
        config_node["full"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["full"]["queue_size"].as<int>())
    );

    mapping_sharp_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["sharp"]["queue_size"].as<int>())
    );

    mapping_flat_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["flat"]["queue_size"].as<int>())
    );

    odom_laser_odometry_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    // path_laser_odometry_pub_ptr_ = std::make_unique<PathPublisher>(
    //     nh, 
    //     config_node["path"]["topic_name"].as<std::string>(),
    //     config_node["path"]["frame_id"].as<std::string>(),
    //     config_node["path"]["queue_size"].as<int>()
    // );    

    return true;
}

bool LaserOdometryFlow::Run(void) {
    // Read inputs
    if (!ReadData()) {
        return false;
    }
    
    // While the data are synced
    while(HasData() && ValidData()) {
        // Update front end odometry
        UpdateData();

        // Publish outputs
        PublishData();
    }

    return true;
}

bool LaserOdometryFlow::ReadData(void) {
    filtered_cloud_sub_ptr_->ParseData(filtered_cloud_buff_);
    corner_points_sharp_sub_ptr_->ParseData(corner_points_sharp_buff_);
    corner_points_less_sharp_sub_ptr_->ParseData(corner_points_less_sharp_buff_);
    surf_points_flat_sub_ptr_->ParseData(surf_points_flat_buff_);
    surf_points_less_flat_sub_ptr_->ParseData(surf_points_less_flat_buff_);

    return true;
}

bool LaserOdometryFlow::HasData(void) {
    if (filtered_cloud_buff_.empty() || 
        corner_points_sharp_buff_.empty() || 
        corner_points_less_sharp_buff_.empty() ||
        surf_points_flat_buff_.empty() ||
        surf_points_less_flat_buff_.empty()) {
        return false;
    }

    return true;
}

bool LaserOdometryFlow::ValidData() {
    const auto& filtered_cloud_time = filtered_cloud_buff_.front().time;

    const auto& corner_points_sharp_time = corner_points_sharp_buff_.front().time;
    const auto& corner_points_less_sharp_time = corner_points_less_sharp_buff_.front().time;
    const auto& surf_points_flat_time = surf_points_flat_buff_.front().time;
    const auto& surf_points_less_flat_time = surf_points_less_flat_buff_.front().time;
    
    // If all the data have the same timestamps
    if ((filtered_cloud_time == corner_points_sharp_time) &&
        (filtered_cloud_time == corner_points_less_sharp_time) && 
        (filtered_cloud_time == surf_points_flat_time) &&
        (filtered_cloud_time == surf_points_less_flat_time)) {
        filtered_cloud_ = std::move(filtered_cloud_buff_.front());
        corner_points_sharp_ = std::move(corner_points_sharp_buff_.front());
        corner_points_less_sharp_ = std::move(corner_points_less_sharp_buff_.front());
        surf_points_flat_ = std::move(surf_points_flat_buff_.front());
        surf_points_less_flat_ = std::move(surf_points_less_flat_buff_.front());

        filtered_cloud_buff_.pop_front();
        corner_points_sharp_buff_.pop_front();
        corner_points_less_sharp_buff_.pop_front();
        surf_points_flat_buff_.pop_front();
        surf_points_less_flat_buff_.pop_front();

        return true;
    }

    return false;
}

bool LaserOdometryFlow::UpdateData(void) {
    laser_odometry_ptr_->Update(
        corner_points_sharp_.cloud_ptr,
        corner_points_less_sharp_.cloud_ptr,
        surf_points_flat_.cloud_ptr,
        surf_points_less_flat_.cloud_ptr,
        odometry_
    );

    return true;
}

bool LaserOdometryFlow::PublishData(void) {
    static int frame_count{0};

    odom_laser_odometry_pub_ptr_->Publish(odometry_, filtered_cloud_.time);
    
    // path_laser_odometry_pub_ptr_->Publish(odometry_, filtered_cloud_.time);

    // If the frame is not to be skipped
    if (0 == (++frame_count % config_.num_frames_skip)) {
        // Reset frame count
        frame_count = 0;
        
        // Publish pointclouds for mapping
        mapping_full_points_pub_ptr_->Publish(filtered_cloud_.cloud_ptr, filtered_cloud_.time);
        mapping_sharp_points_pub_ptr_->Publish(corner_points_less_sharp_.cloud_ptr, filtered_cloud_.time);
        mapping_flat_points_pub_ptr_->Publish(surf_points_less_flat_.cloud_ptr, filtered_cloud_.time);
    }

    return true;
}

} // namespace aloam