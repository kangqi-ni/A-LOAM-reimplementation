#include "aloam_velodyne/aloam/laser_mapping_flow.hpp"

#include "aloam_velodyne/aloam/laser_mapping.hpp"
#include "aloam_velodyne/sensor_data/cloud_data.hpp"
#include "glog/logging.h"

#include "aloam_velodyne/util/file_manager.hpp"
#include "aloam_velodyne/global_definition/global_definition.h"
#include "nav_msgs/Odometry.h"
#include "pcl/common/transforms.h"
#include <Eigen/src/Core/Matrix.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>
#include <mutex>

namespace aloam {

LaserMappingFlow::LaserMappingFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/aloam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    
    // Init params
    InitParam(config_node["laser_mapping"]["param"]);

    // Init subscribers to registered scans
    InitSubscribers(nh, config_node["laser_odometry"]["publisher"]);

    // LOAM front end workflow
    laser_mapping_ptr_ = std::make_unique<LaserMapping>();

    // publisher for point clous used by mapping
    InitPublishers(nh, config_node["laser_mapping"]["publisher"]);
    
    // Start the thread
    process_thread_ = std::thread(std::bind(&LaserMappingFlow::Process, this));
}

bool LaserMappingFlow::InitParam(const YAML::Node& config_node) {
    config_.num_frames_skip = config_node["num_frames_skip"].as<int>();

    return true;
}

void LaserMappingFlow::LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &odom_msg_ptr) {
    // Set the orientation
    Eigen::Quaterniond q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;

    // // buf_.lock();
    // // odom_laser_odometry_buff_.push_back(pose_data);
    // // buf_.unlock();
    
    // Set the translation
    Eigen::Vector3d t(odom_msg_ptr->pose.pose.position.x, 
                      odom_msg_ptr->pose.pose.position.y, 
                      odom_msg_ptr->pose.pose.position.z);

	Eigen::Quaterniond q_w_curr = laser_mapping_ptr_->GetRelativeRotation() * q;
	Eigen::Vector3d t_w_curr = laser_mapping_ptr_->GetRelativeRotation() * t + laser_mapping_ptr_->GetRelativeTranslation(); 

	nav_msgs::Odometry odom_after_mapped;
	odom_after_mapped.header.frame_id = "camera_init";
	odom_after_mapped.child_frame_id = "aft_mapped";
	odom_after_mapped.header.stamp = odom_msg_ptr->header.stamp;
	odom_after_mapped.pose.pose.orientation.x = q_w_curr.x();
	odom_after_mapped.pose.pose.orientation.y = q_w_curr.y();
	odom_after_mapped.pose.pose.orientation.z = q_w_curr.z();
	odom_after_mapped.pose.pose.orientation.w = q_w_curr.w();
	odom_after_mapped.pose.pose.position.x = t_w_curr.x();
	odom_after_mapped.pose.pose.position.y = t_w_curr.y();
	odom_after_mapped.pose.pose.position.z = t_w_curr.z();
	odom_laser_mapping_high_frec_pub_ptr_.publish(odom_after_mapped);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Zero();
    pose.block<3,3>(0,0) = q_w_curr.matrix().cast<float>();
    pose.block<3,1>(0, 3) = t_w_curr.cast<float>();

    // // Publish laser mapping path
    // path_laser_mapping_pub_ptr_->Publish(odometry_, odom_laser_odometry_.time);
}

bool LaserMappingFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    mapping_sharp_points_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["queue_size"].as<int>()
    );

    mapping_flat_points_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["queue_size"].as<int>()
    );

    mapping_full_sub_ptr_ = std::make_unique<CloudSubscriber<CloudData>>(
        nh, 
        config_node["full"]["topic_name"].as<std::string>(), 
        config_node["full"]["queue_size"].as<int>()
    );

    odom_laser_odometry_sub_ptr_ = std::make_unique<OdometrySubscriber>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(), 
        config_node["odometry"]["queue_size"].as<int>()
    );

    odom_laser_odom_sub_ptr_ = nh.subscribe<nav_msgs::Odometry>(
        config_node["odometry"]["topic_name"].as<std::string>(), 
        config_node["odometry"]["queue_size"].as<int>(),
        &LaserMappingFlow::LaserOdometryHandler, this
    );

    return true;
}

bool LaserMappingFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    odom_laser_mapping_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    // path_laser_mapping_pub_ptr_ = std::make_unique<PathPublisher>(
    //     nh,
    //     config_node["path"]["topic_name"].as<std::string>(),
    //     config_node["path"]["frame_id"].as<std::string>(),
    //     config_node["path"]["queue_size"].as<int>()
    // );

    odom_laser_mapping_high_frec_pub_ptr_ = nh.advertise<nav_msgs::Odometry>(
        config_node["odometry_high_frec"]["topic_name"].as<std::string>(),
        config_node["odometry_high_frec"]["queue_size"].as<int>()
    );

    global_map_cloud_pub_ptr_ = std::make_unique<CloudPublisher> (
        nh,
        config_node["global_map"]["topic_name"].as<std::string>(),
        config_node["global_map"]["frame_id"].as<std::string>(),
        config_node["global_map"]["queue_size"].as<int>()
    );

    current_cloud_pub_ptr_ = std::make_unique<CloudPublisher> (
        nh,
        config_node["full_cloud"]["topic_name"].as<std::string>(),
        config_node["full_cloud"]["frame_id"].as<std::string>(),
        config_node["full_cloud"]["queue_size"].as<int>()
    );

    return true;
}

void LaserMappingFlow::Process(void) {
    while(1) {
        while(HasData() && ValidData()) {
            while(!mapping_sharp_points_buff_.empty()) {
                mapping_sharp_points_buff_.pop_front();
                printf("Drop lidar frame in mapping for real time performance\n");
            }

            // Update laser mapping odometry
            UpdateData();

            // Publish data
            PublishData();
        }
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

bool LaserMappingFlow::ReadData(void) {
    buf_.lock(); 
    // Parse registered scans
    mapping_sharp_points_sub_ptr_->ParseData(mapping_sharp_points_buff_);
    mapping_flat_points_sub_ptr_->ParseData(mapping_flat_points_buff_);
    mapping_full_sub_ptr_->ParseData(mapping_full_buff_);

    // Parse laser odometry estimation
    odom_laser_odometry_sub_ptr_->ParseData(odom_laser_odometry_buff_);

    buf_.unlock();
    return true;
}

bool LaserMappingFlow::HasOdomData(void) {
    return !odom_laser_odometry_buff_.empty();
}

bool LaserMappingFlow::HasCloudData(void) {
    return (!mapping_sharp_points_buff_.empty()) && 
           (!mapping_flat_points_buff_.empty()) &&
           (!mapping_full_buff_.empty());
}

bool LaserMappingFlow::HasData(void) {
    buf_.lock();
    if (!(HasOdomData() && HasCloudData())) {
        buf_.unlock();
        return false;
    }

    buf_.unlock();
    return true;
}

bool LaserMappingFlow::ValidData() {
    buf_.lock();
    const auto& odom_scan_to_scan_time = odom_laser_odometry_buff_.front().time;

    const auto& mapping_sharp_points_time = mapping_sharp_points_buff_.front().time;
    const auto& mapping_flat_points_time = mapping_flat_points_buff_.front().time;
    const auto& mapping_full_cloud_time = mapping_full_buff_.front().time;

    if ((odom_scan_to_scan_time < mapping_sharp_points_time)) {
        odom_laser_odometry_buff_.pop_front();
    }

    if ((mapping_flat_points_time < mapping_sharp_points_time)) {
        mapping_flat_points_buff_.pop_front();
    }

    if (mapping_full_cloud_time < mapping_sharp_points_time) {
        mapping_full_buff_.pop_front();
    }

    if ((odom_scan_to_scan_time == mapping_sharp_points_time) &&
        (odom_scan_to_scan_time == mapping_flat_points_time) &&
        odom_scan_to_scan_time == mapping_full_cloud_time) {
        odom_laser_odometry_ = std::move(odom_laser_odometry_buff_.front());
        mapping_sharp_points_ = std::move(mapping_sharp_points_buff_.front());
        mapping_flat_points_ = std::move(mapping_flat_points_buff_.front());
        mapping_full_ = std::move(mapping_full_buff_.front());

        odom_laser_odometry_buff_.pop_front();
        mapping_sharp_points_buff_.pop_front();
        mapping_flat_points_buff_.pop_front();
        mapping_full_buff_.pop_front();

        buf_.unlock();
        return true;
    }

    buf_.unlock();
    return false;
}

bool LaserMappingFlow::UpdateData(void) {
    return laser_mapping_ptr_->Update(
        mapping_sharp_points_.cloud_ptr, mapping_flat_points_.cloud_ptr, 
        odom_laser_odometry_.pose,
        odometry_
    );
}

bool LaserMappingFlow::PublishData(void) {
    static int frame_count{0};

    // Publish laser mapping odometry
    odom_laser_mapping_pub_ptr_->Publish(odometry_, odom_laser_odometry_.time);

    // // Publish laser mapping path
    // path_laser_mapping_pub_ptr_->Publish(odometry_, odom_laser_odometry_.time);

    ++frame_count;

    if ((frame_count % 20) == 0) {
        laser_mapping_ptr_->GetGlobalMap(laser_cloud_map_);
        global_map_cloud_pub_ptr_->Publish(laser_cloud_map_, odom_laser_odometry_.time);
    }

    static CloudData::CloudT::Ptr cloud_pub (new CloudData::CloudT);
    pcl::transformPointCloud(*mapping_full_.cloud_ptr, *cloud_pub, odometry_);
    current_cloud_pub_ptr_->Publish(cloud_pub, odom_laser_odometry_.time);

    return true;
}

} // namespace aloam