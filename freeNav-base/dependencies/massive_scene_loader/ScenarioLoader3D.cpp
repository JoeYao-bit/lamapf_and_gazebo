//
// Created by yaozhuo on 2022/11/27.
//

#include "ScenarioLoader3D.h"
#include <fstream>
using namespace freeNav;

ScenarioLoader3D::ScenarioLoader3D(const std::string& file_path) {

    std::ifstream fin(file_path);
    std::cout << " file path: " << file_path << std::endl;
    if(!fin.is_open()) {
        std::cout << " FATAL:  open " << file_path << " failed " << std::endl;
    }
    std::string line;
    std::vector<std::string> lines;
    int expected_size = 8;
    while (getline(fin, line)) {
        //std::cout << line << std::endl;
        lines.push_back(line);
    }
    for(int i=2; i<lines.size(); i++) {
        std::istringstream sin(lines[i]);
        std::vector<std::string> fields;
        std::string field;
        while (sin.rdbuf()->in_avail() != 0) {
            sin >> field;
            fields.push_back(field);
        }
        if (fields.size() != expected_size) {
            std::cout << " FATAL:  fields.size() " << fields.size() << " != expected_size " << expected_size << std::endl;
            return;
        }
        TestCaseMovingAi current_case;
        for(int i=0; i<3; i++) {
            current_case.test_case_.first[i]  = atof(fields[i].c_str());
            current_case.test_case_.second[i] = atof(fields[i+3].c_str());
        }
        current_case.path_length_   = atof(fields[6].c_str());
        current_case.unknown_value_ = atof(fields[7].c_str());
        case_sequence_.push_back(current_case);
    }
}
