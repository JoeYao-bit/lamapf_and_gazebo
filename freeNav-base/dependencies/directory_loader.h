//
// Created by yaozhuo on 2023/1/19.
//

#ifndef FREENAV_BASE_DIRECTORY_LOADER_H
#define FREENAV_BASE_DIRECTORY_LOADER_H

#include <sys/stat.h>
#include <dirent.h>
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <sstream>

std::set<std::string> getFiles(const char* path);

std::set<std::string> getFilesWithFormat(const char* path, const std::string& format_name);

std::vector<std::string> splitString(const std::string& s, const std::string& c);

std::map<std::string, std::string> convertFromMapToTestConfig(const std::string &map_path);


#endif //FREENAV_DIRECTORY_LOADER_H
