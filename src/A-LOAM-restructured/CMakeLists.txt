cmake_minimum_required(VERSION 3.10.2)
project(aloam_velodyne)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    pcl_ros
    eigen_conversions
    message_generation 
    std_srvs
    cv_bridge
    image_transport
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/OpenCV.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen3.cmake)
include(cmake/sophus.cmake)
include(cmake/ceres.cmake)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

include(cmake/global_definition.cmake)

catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")

list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(scan_registration_node  src/app/scan_registration_node.cpp ${ALL_SRCS})
target_link_libraries(scan_registration_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(laser_odometry_node  src/app/laser_odometry_node.cpp ${ALL_SRCS})
target_link_libraries(laser_odometry_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(laser_mapping_node  src/app/laser_mapping_node.cpp ${ALL_SRCS})
target_link_libraries(laser_mapping_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(kitti_helper_node  src/app/kitti_helper_node.cpp)
target_link_libraries(kitti_helper_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/noetic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
    # scan_registration_node
    # laser_odometry_node
    # laser_mapping_node
    kitti_helper_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)