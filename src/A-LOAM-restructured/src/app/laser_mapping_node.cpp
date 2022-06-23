#include <memory>

#include <ros/ros.h>
#include "glog/logging.h"

#include "aloam_velodyne/global_definition/global_definition.h"

#include "aloam_velodyne/aloam/laser_mapping_flow.hpp"

#include "aloam_velodyne/util/file_manager.hpp"

using namespace aloam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";

    FileManager::CreateDirectory(FLAGS_log_dir);

    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "laser_mapping_node");
    ros::NodeHandle nh;

    // Register back end processing workflow
    std::unique_ptr<LaserMappingFlow> laser_mapping_flow_ptr = std::make_unique<LaserMappingFlow>(nh);

    while (ros::ok()) {
        ros::spinOnce();

        laser_mapping_flow_ptr->ReadData();
    }

    return EXIT_SUCCESS;
}