#include "aloam_velodyne/publisher/path_publisher.hpp"
#include <geometry_msgs/PointStamped.h>

namespace aloam {
    
PathPublisher::PathPublisher(ros::NodeHandle& nh, 
                            std::string topic_name, 
                            std::string frame_id,
                            int buff_size)
    :nh_(nh) {
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    pose_.header.frame_id = frame_id;
    path_.header.frame_id = frame_id;
}

void PathPublisher::Publish(const Eigen::Matrix4f &transform_matrix, double time) {
    ros::Time ros_time(time);
    Publish(transform_matrix, ros_time);
}

void PathPublisher::Publish(const Eigen::Matrix4f &transform_matrix) {
    ros::Time ros_time = ros::Time::now();
    Publish(transform_matrix, ros_time);
}

void PathPublisher::Publish(const Eigen::Matrix4f &transform_matrix, const ros::Time &time) {
    // Create a pose message
    pose_.header.stamp = time;
    pose_.pose.position.x = transform_matrix(0,3);
    pose_.pose.position.y = transform_matrix(1,3);
    pose_.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();

    // Create a path message
    path_.header.stamp = time;
    path_.poses.push_back(pose_);
    publisher_.publish(path_);
}

}