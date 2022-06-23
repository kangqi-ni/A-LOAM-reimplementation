#include "aloam_velodyne/util/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace aloam {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        LOG(WARNING) << "Cannot create file: " << file_path;
        return false;
    }

    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "Cannot create dir: " << directory_path;
        return false;
    }
    return true;
}
}