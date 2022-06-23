#ifndef PATH_PUBLISHER_HPP_
#define PATH_PUBLISHER_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace aloam {
class PathPublisher {
  public:
    PathPublisher(ros::NodeHandle& nh, 
                  std::string topic_name, 
                  std::string frame_id,
                  int buff_size);
    PathPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    geometry_msgs::PoseStamped pose_;
    nav_msgs::Path path_;

    void Publish(const Eigen::Matrix4f& transform_matrix, const ros::Time &time);
};
}
#endif // PATH_PUBLISHER_HPP_