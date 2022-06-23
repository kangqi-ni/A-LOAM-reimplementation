#include "aloam_velodyne/aloam/laser_odometry.hpp"

#include "glog/logging.h"

#include "aloam_velodyne/util/file_manager.hpp"
#include "aloam_velodyne/global_definition/global_definition.h"

#include <limits>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace aloam {

#define DISTORTION 0 // no undistortion for KITTI datasets

LaserOdometry::LaserOdometry(void) {
    // Read yaml config file
    std::string config_file_path = WORK_SPACE_PATH + "/config/aloam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // Set ALOAM front end params
    InitParam(config_node["laser_odometry"]["param"]);

    // Init kdtrees for feature point associations
    InitKdTrees();

    dq_ = Eigen::Quaterniond::Identity();
    dt_ = Eigen::Vector3d::Zero();

    q_ = Eigen::Quaterniond::Identity();
    t_ = Eigen::Vector3d::Zero();
}

bool LaserOdometry::InitParam(const YAML::Node& config_node) {
    config_.scan_period = 0.10;
    
    config_.distance_thresh = config_node["distance_thresh"].as<double>();
    config_.scan_thresh = config_node["scan_thresh"].as<double>();

    config_.num_threads = config_node["num_threads"].as<int>();
    config_.max_num_iteration = config_node["max_num_iteration"].as<int>();
    config_.max_solver_time = config_node["max_solver_time"].as<double>();

    config_.registration_config.set_num_threads(config_.num_threads)
                               .set_max_num_iterations(config_.max_num_iteration)
                               .set_max_solver_time_in_seconds(config_.max_solver_time);
    return true;
}

bool LaserOdometry::InitKdTrees(void) {
    kdtree_.corner.reset(new pcl::KdTreeFLANN<CloudData::PointT>());
    kdtree_.surface.reset(new pcl::KdTreeFLANN<CloudData::PointT>());

    return true;
}

bool LaserOdometry::TransformToStart(const CloudData::PointT &input, CloudData::PointT &output) {
    // Compute the interpolation ratio
    double ratio = 1.0;

    if (DISTORTION) {
        ratio = (input.intensity - int(input.intensity)) / config_.scan_period;
    }
    
    // Transform the point to the start of the frame
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity().slerp(ratio, dq_);
    Eigen::Vector3d dt = ratio * dt_;
    Eigen::Vector3d p(input.x, input.y, input.z);
    Eigen::Vector3d undistorted = dq * p + dt;

    output.x = undistorted.x();
    output.y = undistorted.y();
    output.z = undistorted.z();
    output.intensity = input.intensity;

    return true;
}

bool LaserOdometry::AssociateCornerPoints(
    const CloudData::CloudT &corner_sharp,
    std::vector<CornerPointAssociation> &corner_point_associations) {
    // Find correspondence for corner features
    const int num_query_points = corner_sharp.points.size();

    CloudData::PointT query_point;
    std::vector<int> result_indices;
    std::vector<float> result_squared_distances;
    
    corner_point_associations.clear();
    for (int i = 0; i < num_query_points; ++i) {
        TransformToStart(corner_sharp.points[i], query_point);

        // Find the nearest corner in previous scan
        kdtree_.corner->nearestKSearch(query_point, 1, result_indices, result_squared_distances);

        // If the distance is smaller than the threshold
        if (result_squared_distances[0] < config_.distance_thresh) {
            CornerPointAssociation corner_point_association;

            corner_point_association.query_index = i;
            corner_point_association.ratio = (query_point.intensity - int(query_point.intensity)) / config_.scan_period;

            // Set the first associated point as the closest point:
            corner_point_association.associated_x_index = result_indices[0];

            // Search the second associated point in nearby scans:
            auto get_scan_id = [](const CloudData::PointT &point) { return static_cast<int>(point.intensity); };

            int query_scan_id = get_scan_id(corner_sharp.points[corner_point_association.query_index]);

            float min_distance = std::numeric_limits<float>::infinity();
            int num_candidate_points = kdtree_.candidate_corner_ptr->points.size();
            const auto &candidate_points = kdtree_.candidate_corner_ptr->points;

            // Search in the direction of increasing scan line
            for (int j = corner_point_association.associated_x_index + 1; j < num_candidate_points; ++j) {   
                // Get current scan id
                int curr_scan_id = get_scan_id(candidate_points[j]);

                // If in the same scan line, skip
                if (curr_scan_id <= query_scan_id)
                    continue;

                // If outside nearby scans, stop
                if (curr_scan_id > (query_scan_id + config_.scan_thresh))
                    break;

                // Calculate deviation
                Eigen::Vector3d deviation {
                    candidate_points[j].x - query_point.x,
                    candidate_points[j].y - query_point.y,
                    candidate_points[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // Update associated points
                if (curr_distance < min_distance) {
                    min_distance = curr_distance;
                    corner_point_association.associated_y_index = j;
                }
            }

            // Search in the direction of decreasing scan line
            for (int j = corner_point_association.associated_x_index - 1; j >= 0; --j) {
                // Get current scan id
                int curr_scan_id = get_scan_id(candidate_points[j]);

                // If in the same scan line, skip
                if (curr_scan_id >= query_scan_id)
                    continue;

                // If outside nearby scans, stop
                if (curr_scan_id < (query_scan_id - config_.scan_thresh))
                    break;

                // Calculate deviation
                Eigen::Vector3d deviation {
                    candidate_points[j].x - query_point.x,
                    candidate_points[j].y - query_point.y,
                    candidate_points[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // Update associated points
                if (curr_distance < min_distance) {
                    min_distance = curr_distance;
                    corner_point_association.associated_y_index = j;
                }
            }

            // If the associated indices are valid
            if (corner_point_association.IsValid()) {
                corner_point_associations.push_back(corner_point_association);
            }
        }
    }

    return true;
}

bool LaserOdometry::AssociateSurfacePoints(
    const CloudData::CloudT &surf_flat,
    std::vector<SurfacePointAssociation> &surface_point_associations) {
    const int num_query_points = surf_flat.points.size();

    CloudData::PointT query_point;
    std::vector<int> result_indices;
    std::vector<float> result_squared_distances;
    
    surface_point_associations.clear();
    for (int i = 0; i < num_query_points; ++i) {
        TransformToStart(surf_flat.points[i], query_point);

        // Find nearest surface point in previous scans
        kdtree_.surface->nearestKSearch(query_point, 1, result_indices, result_squared_distances);

        if (result_squared_distances[0] < config_.distance_thresh) {
            SurfacePointAssociation surface_point_association;

            surface_point_association.query_index = i;
            surface_point_association.ratio = (query_point.intensity - int(query_point.intensity)) / config_.scan_period;

            // Set the first associated point as the closest point
            surface_point_association.associated_x_index = result_indices[0];

            // Search the second & third associated points in nearby scans
            auto get_scan_id = [](const CloudData::PointT &point) {return static_cast<int>(point.intensity);};

            int query_scan_id = get_scan_id(surf_flat.points[surface_point_association.query_index]);

            float min_distance_y = std::numeric_limits<float>::infinity();
            float min_distance_z = std::numeric_limits<float>::infinity();
            int num_candidate_points = kdtree_.candidate_surface_ptr->points.size();
            const auto &candidate_points = kdtree_.candidate_surface_ptr->points;

            // Search in the direction of increasing scan line
            for (int j = surface_point_association.associated_x_index + 1; j < num_candidate_points; ++j) {   
                // Get current scan id
                int curr_scan_id = get_scan_id(candidate_points[j]);

                // If outside nearby scans, stop
                if ( curr_scan_id > (query_scan_id + config_.scan_thresh) )
                    break;

                // Calculate deviation
                Eigen::Vector3d deviation{
                    candidate_points[j].x - query_point.x,
                    candidate_points[j].y - query_point.y,
                    candidate_points[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // Update the associated surface point, not above current scan
                if (curr_scan_id <= query_scan_id && curr_distance < min_distance_y) {
                    min_distance_y = curr_distance;
                    surface_point_association.associated_y_index = j;
                }
                // Update the associated surface point, above current scan
                else if (curr_scan_id > query_scan_id && curr_distance < min_distance_z) {
                    min_distance_z = curr_distance;
                    surface_point_association.associated_z_index = j;
                }
            }

            // Search in the direction of decreasing scan line
            for (int j = surface_point_association.associated_x_index - 1; j >= 0; --j)
            {   
                // Get current scan id:
                int curr_scan_id = get_scan_id(candidate_points[j]);

                // If outside nearby scans, stop:
                if (curr_scan_id < (query_scan_id - config_.scan_thresh))
                    break;

                // Calculate deviation:
                Eigen::Vector3d deviation{
                    candidate_points[j].x - query_point.x,
                    candidate_points[j].y - query_point.y,
                    candidate_points[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // Update the associated surface point, not above current scan
                if (curr_scan_id >= query_scan_id && curr_distance < min_distance_y) {
                    min_distance_y = curr_distance;
                    surface_point_association.associated_y_index = j;
                }
                // Update the associated surface point, above current scan
                else if (curr_scan_id < query_scan_id && curr_distance < min_distance_z) {
                    min_distance_z = curr_distance;
                    surface_point_association.associated_z_index = j;
                }
            }

            if (surface_point_association.IsValid()) {
                surface_point_associations.push_back(surface_point_association);
            }
        }
    }

    return true;
}

int LaserOdometry::AddEdgeFactors(
    const CloudData::CloudT &corner_sharp,
    const std::vector<CornerPointAssociation> &corner_point_associations,
    Optimizer &aloam_registration) {
    int num_factors{0};

    for (const auto &corner_point_association: corner_point_associations) {
        Eigen::Vector3d source{
            corner_sharp.points.at(corner_point_association.query_index).x,
            corner_sharp.points.at(corner_point_association.query_index).y,
            corner_sharp.points.at(corner_point_association.query_index).z
        };

        Eigen::Vector3d target_x{
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_x_index).x,
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_x_index).y,
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_x_index).z
        };
    
        Eigen::Vector3d target_y{
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_y_index).x,
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_y_index).y,
            kdtree_.candidate_corner_ptr->points.at(corner_point_association.associated_y_index).z
        };

        aloam_registration.AddEdgeFactor(
            source,
            target_x, target_y,
            1.0 // corner_point_association.ratio
        );

        ++num_factors;
    }

    return num_factors;
}

int LaserOdometry::AddPlaneFactors(
    const CloudData::CloudT &surf_flat,
    const std::vector<SurfacePointAssociation> &surface_point_associations,
    Optimizer &aloam_registration) {
    int num_factors{0};

    for (const auto &surface_point_association: surface_point_associations) {
        Eigen::Vector3d source{
            surf_flat.points.at(surface_point_association.query_index).x,
            surf_flat.points.at(surface_point_association.query_index).y,
            surf_flat.points.at(surface_point_association.query_index).z
        };

        Eigen::Vector3d target_x{
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_x_index).x,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_x_index).y,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_x_index).z
        };
    
        Eigen::Vector3d target_y{
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_y_index).x,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_y_index).y,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_y_index).z
        };

        Eigen::Vector3d target_z{
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_z_index).x,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_z_index).y,
            kdtree_.candidate_surface_ptr->points.at(surface_point_association.associated_z_index).z
        };

        aloam_registration.AddPlaneFactor(
            source,
            target_x, target_y, target_z,
            1.0 // surface_point_association.ratio
        );

        ++num_factors;
    }

    return num_factors;
}

bool LaserOdometry::SetTargetPoints(
    const CloudData::CloudT::Ptr &corner_less_sharp_ptr,
    const CloudData::CloudT::Ptr &surf_less_flat_ptr) {
    kdtree_.candidate_corner_ptr = corner_less_sharp_ptr;
    kdtree_.corner->setInputCloud(kdtree_.candidate_corner_ptr);

    kdtree_.candidate_surface_ptr = surf_less_flat_ptr;
    kdtree_.surface->setInputCloud(kdtree_.candidate_surface_ptr);
    
    if (!inited_) {
        inited_ = true;
    }

    return true;
}

bool LaserOdometry::UpdateOdometry(Eigen::Matrix4f& lidar_odometry) {
    t_ = q_ * dt_ + t_;
    q_ = q_ * dq_;

    lidar_odometry.block<3, 3>(0, 0) = q_.toRotationMatrix().cast<float>();
    lidar_odometry.block<3, 1>(0, 3) = t_.cast<float>();

    return true;
}

bool LaserOdometry::Update(
    CloudData::CloudT::Ptr corner_sharp,
    CloudData::CloudT::Ptr corner_less_sharp,
    CloudData::CloudT::Ptr surf_flat,
    CloudData::CloudT::Ptr surf_less_flat,
    Eigen::Matrix4f& lidar_odometry) {

    if (inited_) {
        // Perform iterative optimizations
        for (int i = 0; i < config_.max_num_iteration; ++i) {
            std::vector<CornerPointAssociation> corner_point_associations;
            AssociateCornerPoints(*corner_sharp, corner_point_associations);

            std::vector<SurfacePointAssociation> surface_point_associations;
            AssociateSurfacePoints(*surf_flat, surface_point_associations);

            // If not enough correspondences
            if (corner_point_associations.size() + surface_point_associations.size() < 10) {
                return false;
            }

            // Build problem
            Optimizer aloam_registration(config_.registration_config, dq_, dt_);
            const auto num_edge_factors = AddEdgeFactors(*corner_sharp, corner_point_associations, aloam_registration);
            const auto num_plane_factors = AddPlaneFactors(*surf_flat, surface_point_associations, aloam_registration);

            // Get relative pose
            aloam_registration.Optimize();
            aloam_registration.GetOptimizedRelativePose(dq_, dt_);
        }

        // Update odometry
        UpdateOdometry(lidar_odometry);
    }

    // Set target feature points for next association:
    SetTargetPoints(corner_less_sharp, surf_less_flat);

    return true;
}

} // namespace aloam