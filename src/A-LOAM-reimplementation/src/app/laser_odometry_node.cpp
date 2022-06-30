#include <memory>

#include <ros/ros.h>
#include "aloam_velodyne/aloam/laser_odometry_flow.hpp"
#include "glog/logging.h"

#include "aloam_velodyne/global_definition/global_definition.h"

#include "aloam_velodyne/aloam/laser_odometry.hpp"

#include "aloam_velodyne/util/file_manager.hpp"

using namespace aloam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";

    FileManager::CreateDirectory(FLAGS_log_dir);

    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "laser_odometry");
    ros::NodeHandle nh;

    // Register front end processing workflow
    std::unique_ptr<LaserOdometryFlow> scan_scan_registration_ptr = std::make_unique<LaserOdometryFlow>(nh);

    // process rate: 10Hz
    while (ros::ok()) {
        ros::spinOnce();

        scan_scan_registration_ptr->Run();
    }

    return EXIT_SUCCESS;
}