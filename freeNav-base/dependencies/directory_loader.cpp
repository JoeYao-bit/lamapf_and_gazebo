//
// Created by yaozhuo on 2023/1/19.
//

#include "directory_loader.h"
#include <string>

std::set<std::string> getFiles(const char* path) {

    const std::string path0 = path;
    DIR* pDir;
    struct dirent* ptr;

    struct stat s;
    lstat(path, &s);

    if(!S_ISDIR(s.st_mode)) {
        std::cout << "not a valid directory: " << path << std::endl;
        return {};
    }

    if(!(pDir = opendir(path))){
        std::cout << "opendir error: " << path << std::endl;
        return {};
    }
    //int i = 0;
    std::string subFile;
    std::set<std::string> files;
    while((ptr = readdir(pDir)) != 0){
        subFile = ptr -> d_name;
        if(subFile == "." || subFile == "..")
            continue;
        subFile = path0 + subFile;
        //std::cout << ++i << ": " << subFile << std::endl;
        files.insert(subFile);
    }
    closedir(pDir);
    return files;
}

std::vector<std::string> splitString(const std::string& s, const std::string& c)
{
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    std::vector<std::string> v;
    while(std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}

std::set<std::string> getFilesWithFormat(const char* path, const std::string& format_name) {
    std::set<std::string> files = getFiles(path);
    std::set<std::string> filtered_files;
    for(const auto& file : files) {
        auto splitted_file = splitString(file, std::string("."));
        if(splitted_file.back() == format_name) {
            //std::cout << " splitted_file.back() " << splitted_file.back() << " / format_name " << format_name << std::endl;
            filtered_files.insert(file);
        }
    }
    return filtered_files;
}

// convert the map path to a test config that all in the same directory
std::map<std::string, std::string> convertFromMapToTestConfig(const std::string &map_path) {
//    SingleMapTestConfig<2> MapTestConfig_maze512_4_0 =
//
//            {
//                    {"map_name",    "maze-512-4"},
//                    {"map_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map/maze512-4-0.map"},
//                    {"vis_path",    "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-vis/maze512-4-0_ENLSVG.vis"},
//                    {"config_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-scene/maze512-4-0.map.scen"},
//                    {"output_path", "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-output/maze512-4-0.txt"}
//            };

    auto splitted_file = splitString(map_path, std::string("."));
    std::map<std::string, std::string> mapTestConfig;
    std::string prefix;
    std::stringstream ss;
    for(int i=0; i<splitted_file.size()-1; i++) {
        ss << splitted_file[i];
    }
    prefix = ss.str();
    mapTestConfig.insert({std::string("map_name"),    map_path});
    mapTestConfig.insert({std::string("map_path"),    map_path});
    mapTestConfig.insert({std::string("vis_path"),    prefix + ".vis"});
    mapTestConfig.insert({std::string("config_path"), prefix + ".map.scen"});
    mapTestConfig.insert({std::string("output_path"), prefix + ".txt"});

    return mapTestConfig;
}





















