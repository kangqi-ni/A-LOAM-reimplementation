#ifndef SUB_MAP_HPP_
#define SUB_MAP_HPP_

#include <memory>

#include <vector>
#include <set>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "aloam_velodyne/sensor_data/cloud_data.hpp"
#include "aloam_velodyne/cloud_filter/voxel_filter.hpp"

namespace aloam {

class SubMap {
public:
    class Config {
    public:
        double resolution{50.0};

        int num_tiles_x{21};
        int num_tiles_y{21};
        int num_tiles_z{11};

        int num_tiles;

        int reanchor_margin{3};
        int local_map_radius{2};

        Config(void) {}

        Config& set_resolution(const double &resolution) {
            this->resolution = resolution;
            return *this;
        }

        Config& set_num_tiles_x(const int num_tiles_x) {
            this->num_tiles_x = num_tiles_x;
            return *this;
        }

        Config& set_num_tiles_y(const int num_tiles_y) {
            this->num_tiles_y = num_tiles_y;
            return *this;
        }

        Config& set_num_tiles_z(const int num_tiles_z) {
            this->num_tiles_z = num_tiles_z;
            return *this;
        }

        Config& set_num_tiles(const int num_tiles) {
            this->num_tiles = num_tiles;
            return *this;
        }

        Config& set_reanchor_margin(const int reanchor_margin) {
            this->reanchor_margin = reanchor_margin;
            return *this;
        }

        Config& set_local_map_radius(const int local_map_radius) {
            this->local_map_radius = local_map_radius;
            return *this;
        }
    };

    SubMap(const Config& config);

    struct Index {
        int x;
        int y;
        int z;

        Index(void) : x{-1}, y{-1}, z{-1} {}
        
        Index(int x, int y, int z) {
            this->x = x;
            this->y = y;
            this->z = z;
        }
    };

    struct LocalMap {
        Index query_index;
        CloudData::CloudT::Ptr sharp;
        CloudData::CloudT::Ptr flat;
    };

    LocalMap GetLocalMap(
        const Eigen::Vector3d &query_position
    );

    bool RegisterLineFeaturePoints(
        const CloudData::CloudT::Ptr points, 
        const Eigen::Quaterniond& q, const Eigen::Vector3d& t
    );
    bool RegisterPlaneFeaturePoints(
        const CloudData::CloudT::Ptr points, 
        const Eigen::Quaterniond& q, const Eigen::Vector3d& t
    );

    bool DownsampleSubMap(
        std::unique_ptr<VoxelFilter>& sharp_filter_ptr,
        std::unique_ptr<VoxelFilter>& flat_filter_ptr
    );

    void FormGlobalMap(CloudData::CloudT::Ptr &cloud_ptr);

private:
    Config config_;

    Index center_;

    struct {
        std::vector<CloudData::CloudT::Ptr> sharp;
        std::vector<CloudData::CloudT::Ptr> flat;
    } tiles_;

    struct {
        std::set<size_t> sharp;
        std::set<size_t> flat;
    } recently_accessed_;

    ///@brief odometry frame position to tile index (x, y, z)
    bool IsValidIndex(const Index& index);
    Index GetTileIndex(const Eigen::Vector3d &t);
    Index GetTileIndex(const CloudData::PointT &point);

    ///@brief tile index (x, y, z) to access id
    int GetTileId(const Index& index);
    int GetTileId(const int x, const int y, const int z);

    ///@brief shift along x:
    void ShiftForwardX(void);
    void ShiftBackwardX(void);
    ///@brief shift along y:
    void ShiftForwardY(void);
    void ShiftBackwardY(void);
    ///@brief shift along z:
    void ShiftForwardZ(void);
    void ShiftBackwardZ(void);

    ///@brief sync sub map with query position:
    void Reanchor(Index &query_index);
    ///@brief get local map:
    LocalMap GetLocalMap(
        const Index& query_index
    );

    bool ProjectToMapFrame(
        const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
        const CloudData::CloudT::Ptr& source,
        CloudData::CloudT::Ptr& target
    );

    bool RegisterFeaturePoints(
        const CloudData::CloudT::Ptr points, 
        const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
        std::vector<CloudData::CloudT::Ptr> &tiles,
        std::set<size_t>& recently_accessed
    );
};

inline bool operator==(const SubMap::Index& lhs, const SubMap::Index& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

} // namespace aloam

#endif // SUB_MAP_HPP_