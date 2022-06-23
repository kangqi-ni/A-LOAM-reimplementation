#ifndef POSE_DATA_HPP_
#define POSE_DATA_HPP_

#include <Eigen/Dense>

namespace aloam {

class PoseData {
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    
  public:
    Eigen::Quaternionf GetQuaternion();
};

} // namespace aloam

#endif // POSE_DATA_HPP_