#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace aloam {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
} // aloam

#endif // FILE_MANAGER_HPP_
