//
// Created by yaozhuo on 2024/6/28.
//

#ifndef EXE_COMMON_INTERFACES_H
#define EXE_COMMON_INTERFACES_H
#pragma once

#include <iostream>
#include <cmath>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

#include "LA-MAPF/algorithm/LA-MAPF/circle_shaped_agent.h"
#include "LA-MAPF/algorithm/LA-MAPF/block_shaped_agent.h"

#include "LA-MAPF/algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "LA-MAPF/algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "LA-MAPF/algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "LA-MAPF/algorithm/LA-MAPF/instance_serialize_and_deserialize.h"
#include "LA-MAPF/algorithm/precomputation_for_decomposition.h"

#include "LA-MAPF/algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "LA-MAPF/algorithm/break_loop_decomposition/break_loop_decomposition.h"

#include "freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "freeNav-base/dependencies/2d_grid/picture_loader.h"

#include "test_data.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
#include "LA-MAPF/algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"

#include "LA-MAPF/algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "freeNav-base/dependencies/memory_analysis.h"
#include "freeNav-base/visualization/canvas/canvas.h"


#include <yaml-cpp/yaml.h>



//#include "path_execution.hpp"
using std::placeholders::_1;

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

int zoom_ratio = 10;



std::string pkg_path = "..";


SingleMapTestConfig<2> MAPFTestConfig_LargeOfficeEnv =
{
        {"map_name",     "LargeOfficeEnv"},
        {"map_path",     pkg_path+"/world/map/map_large_office_white.png"},
        {"la_ins_path",  pkg_path+"/world/map/map_large_office.txt"},
};

SingleMapTestConfig<2> MAPFTestConfig_LargeOfficeEnvSecond =
{
        {"map_name",     "LargeOfficeEnvSecond"},
        {"map_path",     pkg_path+"/world/map/map_large_office_white_second.png"},
        {"la_ins_path",  pkg_path+"/world/map/map_large_office_second.txt"},
};


// my_map_260225

SingleMapTestConfig<2> MAPFTestConfig_B112 =
{
        {"map_name",     "B112"},
        {"map_path",     pkg_path+"/world/map/my_map_260225.pgm"},
        {"la_ins_path",  pkg_path+"/world/map/my_map_260225.txt"},
};

SingleMapTestConfig<2> MAPFTestConfig_IndustrialWarehouse =
{
        {"map_name",     "IndustrialWarehouse"},
        {"map_path",     pkg_path+"/world/map/industrial_warehouse_white.png"},
        {"la_ins_path",  pkg_path+"/world/map/industrial_warehouse_white.txt"},
};

std::map<std::string, std::string> demoMapYaml = {
        {"map_name",       "demo"},
        {"map_path",       pkg_path+"/map/my_map.pgm"},
        {"la_ins_path",    pkg_path+"/map/my_map_ins.txt"},
        {"yaml_file_path", pkg_path+"/map/my_map.yaml"}
};

//std::string map_path_pic = "/home/yaozhuo/code/ros2_ws/src/large_agent_mapf/world/map/map_large_office.png";
//std::string ins_path_pic = "/home/yaozhuo/code/ros2_ws/src/large_agent_mapf/world/map/map_large_office.scen";

double reso = 0.1; // how long a grid occupied in real world ?

// following file path related to .world file office_env_large.world

// MAPFTestConfig_IndustrialWarehouse (ok)
// MAPFTestConfig_LargeOfficeEnv
// MAPFTestConfig_LargeOfficeEnvSecond
// MAPFTestConfig_B112

auto map_test_config = MAPFTestConfig_IndustrialWarehouse;//MAPFTestConfig_LargeOfficeEnv

auto is_grid_occupied = [](const cv::Vec3b& color) -> bool {
    if (color == cv::Vec3b::all(255)) { return false; }
    return true;
};


// ros2 generated pgm map
auto is_grid_occupied_pgm = [](const cv::Vec3b& color) -> bool {
    if (color[0]<240 || color[1]<240 || color[2] < 240) { return true; }
    return false;
};

// load map
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied);

DimensionLength* dim = loader.getDimensionInfo();

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

// test_count: the total count of start and target pair in the scenario file
// required_count: required
std::vector<std::set<int> > pickCasesFromScene(int test_count,
                                               const std::vector<int>& required_counts,
                                               int instance_count) {
    std::vector<std::set<int> > retv;
    for(int i=0; i<instance_count; i++) {
        for(const int& required_count : required_counts) {
            std::set<int> instance;
            while(1) {
                int current_pick = rand() % test_count;
                if(instance.find(current_pick) == instance.end()) {
                    instance.insert(current_pick);
                    if(instance.size() == required_count) {
                        retv.push_back(instance);
                        break;
                    }
                }
            }
        }
    }
    return retv;
}




bool reachPosition(const float& x, const float& y, const float& target_x, const float& target_y) {
    float dist_to_target_position = (Pointf<2>{x, y} - Pointf<2>{target_x, target_y}).Norm();
    return fabs(dist_to_target_position) < 0.05;                                            
}

float shortestAngularDistance(float a, float b) {
    double d = fmod(b - a + M_PI, 2.0 * M_PI);
    if (d < 0)
        d += 2.0 * M_PI;
    return d - M_PI;
    return d;
}


bool reachOrientation(const float& orientation, const float& target_orientation) {
    float angle_to_target = shortestAngularDistance(orientation, target_orientation);
    // return fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001;
    return fabs(angle_to_target) < M_PI*6./180.;
}

bool reachTarget(const Pointf<3>& cur_pose, const Pointf<3>& target_ptf) {
    return reachPosition(cur_pose[0], cur_pose[1], target_ptf[0], target_ptf[1]) && reachOrientation(cur_pose[2], target_ptf[2]);
}

bool paused = true;

#endif //LAYEREDMAPF_COMMON_INTERFACES_H
