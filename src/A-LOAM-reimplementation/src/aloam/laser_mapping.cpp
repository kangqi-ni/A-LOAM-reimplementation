#include "aloam_velodyne/aloam/laser_mapping.hpp"

#include "glog/logging.h"

#include "aloam_velodyne/util/file_manager.hpp"
#include "aloam_velodyne/global_definition/global_definition.h"

#include <chrono>

#include <limits>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

namespace aloam {

LaserMapping::LaserMapping(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/aloam.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // init matching config:
    InitParams(config_node["laser_mapping"]["param"]["matching"]);

    // init filters:
    InitFilters(config_node["laser_mapping"]["param"]["filter"]);

    // init submap:
    InitKdTrees();
    InitSubMap(config_node["laser_mapping"]["param"]["submap"]);
}

bool LaserMapping::Update(
    const CloudData::CloudT::Ptr sharp_points,
    const CloudData::CloudT::Ptr flat_points,
    const Eigen::Matrix4f& odom_scan_to_scan,
    Eigen::Matrix4f& lidar_odometry) {
    // Down sample feature points
    CloudData::CloudT::Ptr filtered_sharp_points(new CloudData::CloudT());
    filter_.sharp_filter_ptr_->Filter(sharp_points, filtered_sharp_points);
    CloudData::CloudT::Ptr filtered_flat_points(new CloudData::CloudT());
    filter_.flat_filter_ptr_->Filter(flat_points, filtered_flat_points);

    // Predict scan-map odometry
    PredictScanMapOdometry(odom_scan_to_scan);
    
    // Get local map:
    auto local_map = submap_ptr_->GetLocalMap(pose_.scan_map_odometry.t);

    // If sufficient feature points for matching have been found:
    if (HasSufficientFeaturePoints(local_map)) {
        // Set targets
        SetTargetPoints(local_map);

        // Perform iterative optimization:
        // LOG(WARNING) << "Scan-Map Registration: " << std::endl;
        for (int i = 0; i < config_.max_num_iteration; ++i) {
            // Build problem
            Optimizer aloam_registration(
                config_.registration_config,
                pose_.scan_map_odometry.q, pose_.scan_map_odometry.t
            );
            AddEdgeFactors(filtered_sharp_points, local_map.sharp, aloam_registration);
            AddPlaneFactors(filtered_flat_points, local_map.flat, aloam_registration);

            // Get relative pose:
            aloam_registration.Optimize();
            aloam_registration.GetOptimizedRelativePose(pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);
        }
    }

    // Update relative pose estimation
    UpdateRelativePose();

    // Register feature points
    submap_ptr_->RegisterLineFeaturePoints(filtered_sharp_points, pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);
    submap_ptr_->RegisterPlaneFeaturePoints(filtered_flat_points, pose_.scan_map_odometry.q, pose_.scan_map_odometry.t);

    // Downsample local map
    static int dowmsample_count{0};
    if (0 == ++dowmsample_count % 1) {
        submap_ptr_->DownsampleSubMap(
            filter_.sharp_filter_ptr_, filter_.flat_filter_ptr_
        );

        dowmsample_count = 0;
    }

    // Update odometry
    UpdateOdometry(lidar_odometry);

    return true;
}

void LaserMapping::GetGlobalMap(CloudData::CloudT::Ptr &cloud_ptr) {
    submap_ptr_->FormGlobalMap(cloud_ptr);
}

bool LaserMapping::InitParams(const YAML::Node& config_node) {
    config_.min_num_sharp_points = config_node["min_num_sharp_points"].as<int>();
    config_.min_num_flat_points = config_node["min_num_flat_points"].as<int>();
    
    config_.distance_thresh = config_node["distance_thresh"].as<double>();

    config_.num_threads = config_node["num_threads"].as<int>();
    config_.max_num_iteration = config_node["max_num_iteration"].as<int>();
    config_.max_solver_time = config_node["max_solver_time"].as<double>();

    config_.registration_config.set_num_threads(config_.num_threads)
                               .set_max_num_iterations(config_.max_num_iteration)
                               .set_max_solver_time_in_seconds(config_.max_solver_time);

    return true;
}

bool LaserMapping::InitFilters(const YAML::Node& config_node) {
    filter_.sharp_filter_ptr_ = std::make_unique<VoxelFilter>(config_node["sharp"]);
    filter_.flat_filter_ptr_ = std::make_unique<VoxelFilter>(config_node["flat"]);

    return true;
}

bool LaserMapping::InitKdTrees(void) {
    kdtree_.sharp.reset(new pcl::KdTreeFLANN<CloudData::PointT>());
    kdtree_.flat.reset(new pcl::KdTreeFLANN<CloudData::PointT>());

    return true;
}

bool LaserMapping::InitSubMap(const YAML::Node& config_node) {
    // init submap config:
    aloam::SubMap::Config config;

    config.set_resolution(config_node["resolution"].as<double>())
          .set_num_tiles_x(config_node["num_tiles_x"].as<int>())
          .set_num_tiles_y(config_node["num_tiles_y"].as<int>())
          .set_num_tiles_z(config_node["num_tiles_z"].as<int>())
          .set_num_tiles(config_node["num_tiles"].as<int>())
          .set_reanchor_margin(config_node["reanchor_margin"].as<int>())
          .set_local_map_radius(config_node["local_map_radius"].as<int>());
    
    // init submap:
    submap_ptr_ = std::make_unique<aloam::SubMap>(config);

    return true;
}

bool LaserMapping::HasSufficientFeaturePoints(const aloam::SubMap::LocalMap &local_map) {
    const auto num_sharp_points = static_cast<int>(local_map.sharp->points.size());
    const auto num_flat_points = static_cast<int>(local_map.flat->points.size());

    return (
        (num_sharp_points > config_.min_num_sharp_points) && 
        (num_flat_points > config_.min_num_flat_points)
    );
}

bool LaserMapping::SetTargetPoints(
    aloam::SubMap::LocalMap& local_map) {
    filter_.sharp_filter_ptr_->Filter(local_map.sharp, local_map.sharp);
    filter_.flat_filter_ptr_->Filter(local_map.flat, local_map.flat);

    kdtree_.sharp->setInputCloud(local_map.sharp);
    kdtree_.flat->setInputCloud(local_map.flat);

    return true;
}

bool LaserMapping::ProjectToMapFrame(
    const CloudData::CloudT::Ptr& source,
    CloudData::CloudT::Ptr& query) {
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    transform_matrix.block<3, 3>(0, 0) = pose_.scan_map_odometry.q.toRotationMatrix().cast<float>();
    transform_matrix.block<3, 1>(0, 3) = pose_.scan_map_odometry.t.cast<float>();
    
    pcl::transformPointCloud(*source, *query, transform_matrix);

    return true;
}

int LaserMapping::AddEdgeFactors(
    const CloudData::CloudT::Ptr source,
    const CloudData::CloudT::Ptr target,
    Optimizer &aloam_registration ) {
    const auto num_feature_points = source->points.size();

    CloudData::CloudT::Ptr query(new CloudData::CloudT());
    ProjectToMapFrame(source, query);

    std::vector<int> target_candidate_indices;
    std::vector<float> target_candidate_distances;

    int num_factors{0};
    for (size_t i = 0; i < num_feature_points; ++i) {
        const auto& feature_point_in_lidar_frame = source->points[i];
        const auto& feature_point_in_map_frame = query->points[i];

        // Search in target cloud
        const int num_neighbors = 5;
        kdtree_.sharp->nearestKSearch(feature_point_in_map_frame, num_neighbors, target_candidate_indices, target_candidate_distances);
        if (target_candidate_distances.back() < config_.distance_thresh) {
            // Estimate line direction using Eigen decomposition
            Eigen::Vector3d mu = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

            for (size_t i = 0; i < num_neighbors; ++i) {
                const auto& target_point = target->points[target_candidate_indices[i]];

                Eigen::Vector3d x{
                    target_point.x,
                    target_point.y,
                    target_point.z
                };

                mu += x;
                cov += x*x.transpose();
            }

            mu = (1.0/num_neighbors) * mu;
            cov = (1.0/num_neighbors) * cov - mu*mu.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);

            // Check goodness of fitness
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {   
                const auto &unit_direction = saes.eigenvectors().col(2);

                // source in lidar frame
                const Eigen::Vector3d source{
                    feature_point_in_lidar_frame.x,
                    feature_point_in_lidar_frame.y,
                    feature_point_in_lidar_frame.z
                };

                // target in map frame
                const auto target_x =  0.1 * unit_direction + mu;
                const auto target_y = -0.1 * unit_direction + mu;

                // Use predicted odometry from laser odometry as initial value:
                aloam_registration.AddEdgeFactor(
                    source,
                    target_x, target_y,
                    1.0
                );

                ++num_factors;
            }	
        }
    }

    return num_factors;
}

int LaserMapping::AddPlaneFactors(
    const CloudData::CloudT::Ptr source,
    const CloudData::CloudT::Ptr target,
    Optimizer &aloam_registration ) {
    const auto num_feature_points = source->points.size();

    CloudData::CloudT::Ptr query(new CloudData::CloudT());
    ProjectToMapFrame(source, query);

    std::vector<int> target_candidate_indices;
    std::vector<float> target_candidate_distances;

    int num_factors{0};
    for (size_t i = 0; i < num_feature_points; ++i) { 
        const auto& feature_point_in_lidar_frame = source->points[i];
        const auto& feature_point_in_map_frame = query->points[i];

        // Search in target
        const int num_neighbors = 5;
        kdtree_.sharp->nearestKSearch(feature_point_in_map_frame, num_neighbors, target_candidate_indices, target_candidate_distances);
        if (target_candidate_distances.back() < config_.distance_thresh) {
            // Estimate plane normal direction using least square
            Eigen::MatrixXd X = Eigen::MatrixXd::Zero(num_neighbors, 3);
            Eigen::MatrixXd b = Eigen::MatrixXd::Constant(num_neighbors, 1, -1.0);

            for (size_t i = 0; i < num_neighbors; ++i) {
                const auto &target_point = target->points[target_candidate_indices[i]];

                X(i, 0) = target_point.x;
                X(i, 1) = target_point.y;
                X(i, 2) = target_point.z;
            }

            Eigen::Vector3d norm = X.colPivHouseholderQr().solve(b);
            const double negative_oa_dot_norm = norm.norm();
            norm.normalize();

            // Check goodness of fitness
            bool is_plane_valid = true;
            for (size_t i = 0; i < num_neighbors; ++i) {
                const auto &target_point = target->points[target_candidate_indices[i]];
				if (fabs(norm(0) * target_point.x +
						 norm(1) * target_point.y +
						 norm(2) * target_point.z + negative_oa_dot_norm) > 0.2) {
					is_plane_valid = false;
					break;
				}
            }

            if (is_plane_valid) {
                // source in lidar frame
                const Eigen::Vector3d source{
                    feature_point_in_lidar_frame.x,
                    feature_point_in_lidar_frame.y,
                    feature_point_in_lidar_frame.z
                };

                // Use predicted odometry from laser odometry as initial value:
                aloam_registration.AddPlaneNormFactor(
                    source,
                    norm, negative_oa_dot_norm
                );

                ++num_factors;
            }
        }
    }

    return num_factors;
}

bool LaserMapping::PredictScanMapOdometry(const Eigen::Matrix4f& odom_scan_to_scan) {
    // Set laser odometry pose estimate
    pose_.scan_scan_odometry.q = odom_scan_to_scan.block<3, 3>(0, 0).cast<double>();
    pose_.scan_scan_odometry.t = odom_scan_to_scan.block<3, 1>(0, 3).cast<double>();

    // Predict laser mapping estimation
    const auto& dq = pose_.relative.q;
    const auto& dt = pose_.relative.t;

    pose_.scan_map_odometry.q = dq*pose_.scan_scan_odometry.q;
    pose_.scan_map_odometry.t = dq*pose_.scan_scan_odometry.t + dt;

    return true;
}

bool LaserMapping::UpdateRelativePose(void) {
    // Update relative pose for next round estimation
    pose_.relative.q = pose_.scan_map_odometry.q * pose_.scan_scan_odometry.q.inverse();
    pose_.relative.t = pose_.scan_map_odometry.t - pose_.relative.q * pose_.scan_scan_odometry.t;

    return true;
}

bool LaserMapping::UpdateOdometry(Eigen::Matrix4f& lidar_odometry) {
    lidar_odometry.block<3, 3>(0, 0) = pose_.scan_map_odometry.q.toRotationMatrix().cast<float>();
    lidar_odometry.block<3, 1>(0, 3) = pose_.scan_map_odometry.t.cast<float>();

    return true;
}

} // namespace aloam