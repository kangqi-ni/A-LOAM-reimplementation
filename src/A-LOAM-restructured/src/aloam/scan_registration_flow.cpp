#include "aloam_velodyne/aloam/scan_registration.hpp"
#include "aloam_velodyne/aloam/scan_registration_flow.hpp"

#include "glog/logging.h"

#include "aloam_velodyne/util/file_manager.hpp"
#include "aloam_velodyne/global_definition/global_definition.h"

namespace aloam {

ScanRegistrationFlow::ScanRegistrationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/aloam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // Create subscribers to raw Velodyne measurements
    InitSubscribers(nh, config_node["scan_registration"]["subscriber"]);

    // InitMotionCompensator();

    scan_registration_ptr = std::make_unique<ScanRegistration>();

    filtered_cloud_data_.reset(new CloudData::CloudT());
    corner_sharp_.reset(new CloudData::CloudT());
    corner_less_sharp_.reset(new CloudData::CloudT());
    surf_flat_.reset(new CloudData::CloudT());
    surf_less_flat_.reset(new CloudData::CloudT());

    // Create publishers
    InitPublishers(nh, config_node["scan_registration"]["publisher"]);
}

bool ScanRegistrationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    // velocity_sub_ptr_ = std::make_unique<VelocitySubscriber>(
    //     nh, 
    //     config_node["rtk"]["topic_name"].as<std::string>(), 
    //     config_node["rtk"]["queue_size"].as<int>()
    // );

    // lidar_to_imu_ptr_ = std::make_unique<TFListener>(
    //     nh, 
    //     config_node["tf"]["frame_id"].as<std::string>(), 
    //     config_node["tf"]["child_frame_id"].as<std::string>()
    // );

    cloud_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["velodyne"]["topic_name"].as<std::string>(), 
        config_node["velodyne"]["queue_size"].as<int>()
    );

    return true;
}

// bool ScanRegistrationFlow::InitMotionCompensator(void) {
//     motion_compensator_ptr_ = std::make_unique<DistortionAdjust>();

//     return true;
// }

bool ScanRegistrationFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    // Create publishers
    // filtered cloud
    filtered_cloud_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["filtered"]["queue_size"].as<int>())
    );
    // sharp corner features
    corner_points_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["sharp"]["queue_size"].as<int>())
    );
    // less sharp corner featuress
    corner_points_less_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_sharp"]["queue_size"].as<int>())
    );
    // flat surface features
    surf_points_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["flat"]["queue_size"].as<int>())
    );
    // less flat surface features
    surf_points_less_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_flat"]["queue_size"].as<int>())
    );
    // pointcloud without invalid and close points
    removed_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["removed"]["topic_name"].as<std::string>(), 
        config_node["removed"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["removed"]["queue_size"].as<int>())
    );

    return true;
}

bool ScanRegistrationFlow::Run(void) {
    // if (!InitCalibration()) {
    //     // LOG(WARNING) << "DataPretreatFlow: InitCalibration..." << std::endl;
    //     return false;
    // }

    if (!ReadData()) {
        return false;
    }

    while( HasData() && ValidData()) {
        // Update velodyne measurements
        UpdateData();

        // Publish data
        PublishData();
    }

    return true;
}

// bool ScanRegistrationFlow::InitCalibration(void) {
//     // lookup imu pose in lidar frame:
//     static bool calibration_received = false;
//     if (!calibration_received) {
//         if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
//             calibration_received = true;
//         }
//     }

//     return calibration_received;
// }

bool ScanRegistrationFlow::ReadData(void) {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // velocity_sub_ptr_->ParseData(raw_velocity_data_buff_);

    return true;
}

bool ScanRegistrationFlow::HasData(void) {
    // if ( cloud_data_buff_.empty() || raw_velocity_data_buff_.empty() ) {
    if (cloud_data_buff_.empty()) {
        return false;
    }

    return true;
}

bool ScanRegistrationFlow::ValidData() {
    // use timestamp of lidar measurement as reference:
    const auto cloud_time = cloud_data_buff_.front().time;
    // const auto velocity_time = raw_velocity_data_buff_.front().time;

    // if (std::fabs(cloud_time - velocity_time) > 0.10) {
    //     return false;
    // }

    // current_cloud_data_ = std::move(cloud_data_buff_.front());
    // current_velocity_data_ = std::move(raw_velocity_data_buff_.front());

    // cloud_data_buff_.pop_front();
    // raw_velocity_data_buff_.pop_front();

    // bool valid_velocity = VelocityData::SyncData(raw_velocity_data_buff_, synced_velocity_data_buff_, cloud_time);

    // if (!valid_velocity) {
    //     if (!raw_velocity_data_buff_.empty() && raw_velocity_data_buff_.front().time > cloud_time) {
    //         cloud_data_buff_.pop_front();
    //     }
    //     return false;
    // }

    current_cloud_data_ = std::move(cloud_data_buff_.front());
    // current_velocity_data_ = std::move(synced_velocity_data_buff_.front());

    cloud_data_buff_.pop_front();
    // synced_velocity_data_buff_.pop_front();

    return true;
}

bool ScanRegistrationFlow::UpdateData(void) {
    //
    // TO-BE-EVALUATED-BY-MAINTAINER: first apply motion compensation:
    // 
    // current_velocity_data_.TransformCoordinate(lidar_to_imu_);

    // motion_compensator_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    // motion_compensator_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    // Extract scan lines
    scan_registration_ptr->Update(
        current_cloud_data_, 
        filtered_cloud_data_,
        corner_sharp_,
        corner_less_sharp_,
        surf_flat_,
        surf_less_flat_
    );

    return true;
}

bool ScanRegistrationFlow::PublishData(void) {
    // Publish pointclouds
    filtered_cloud_pub_ptr_->Publish(filtered_cloud_data_, current_cloud_data_.time);

    // Publish features 
    corner_points_sharp_pub_ptr_->Publish(corner_sharp_, current_cloud_data_.time);
    corner_points_less_sharp_pub_ptr_->Publish(corner_less_sharp_, current_cloud_data_.time);
    surf_points_flat_pub_ptr_->Publish(surf_flat_, current_cloud_data_.time);
    surf_points_less_flat_pub_ptr_->Publish(surf_less_flat_, current_cloud_data_.time);

    return true;
}

} // namespace aloam