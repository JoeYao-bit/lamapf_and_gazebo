//
// Created by yaozhuo on 2024/6/28.
//

#ifndef EXE_COMMON_INTERFACES_H
#define EXE_COMMON_INTERFACES_H
#pragma once
#include "rclcpp/rclcpp.hpp"
#include <iostream>
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
#include "test_data.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "LA-MAPF/algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "freeNav-base/dependencies/memory_analysis.h"

#include <tf2/LinearMath/Quaternion.h>
// #include "large_agent_mapf/srv/path_execution.hpp" // 存在于install目录但找不到,cmakelist中自己包括自己就找到了
#include "std_msgs/msg/string.hpp"


//#include "path_execution.hpp"
using std::placeholders::_1;

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

int zoom_ratio = 10;

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;
bool draw_all_instance = false;
bool draw_heuristic_table = false;
bool draw_heuristic_table_ignore_rotate = false;

bool draw_path = true;
bool draw_full_path = true;
bool draw_visit_grid_table = false;

SingleMapTestConfig<2> MAPFTestConfig_LargeOfficeEnv =
{
        {"map_name",     "LargeOfficeEnv"},
        {"map_path",     "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/world/map/map_large_office_white.png"},
        {"la_ins_path", "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/world/map/map_large_office.txt"},
};

SingleMapTestConfig<2> MAPFTestConfig_LargeOfficeEnvSecond =
{
        {"map_name",     "LargeOfficeEnvSeccond"},
        {"map_path",     "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/world/map/map_large_office_white_second.png"},
        {"la_ins_path", "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/world/map/map_large_office_second.txt"},
};

//std::string map_path_pic = "/home/yaozhuo/code/ros2_ws/src/large_agent_mapf/world/map/map_large_office.png";
//std::string ins_path_pic = "/home/yaozhuo/code/ros2_ws/src/large_agent_mapf/world/map/map_large_office.scen";

double reso = 0.1; // how long a grid occupied in real world ?

// following file path related to .world file office_env_large.world
auto map_test_config = MAPFTestConfig_LargeOfficeEnvSecond;//MAPFTestConfig_LargeOfficeEnv

auto is_grid_occupied = [](const cv::Vec3b& color) -> bool {
    if (color == cv::Vec3b::all(255)) { return false; }
    return true;
};

// load map
PictureLoader loader(map_test_config.at("map_path"), is_grid_occupied);

DimensionLength* dim = loader.getDimensionInfo();

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

/*
 *     auto layered_paths = layeredLargeAgentMAPF<2, AgentType>(deserializer.getInstances(),
                                                             deserializer.getAgents(),
                                                             dim, is_occupied,
                                                             CBS::LargeAgentCBS_func<2, AgentType >,
                                                             grid_visit_count_table,
                                                             30, decomposer_ptr,
                                                             true);
 * */

template<typename MethodType>
void loadInstanceAndPlanning(const std::string& file_path, double time_limit = 30) {
    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    auto start_t = clock();
    MethodType method(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);
    auto init_t = clock();
    double init_time_cost = (((double)init_t - start_t)/CLOCKS_PER_SEC);
    if(init_time_cost >= time_limit) {
        std::cout << "NOTICE: init of large agent MAPF instance run out of time" << std::endl;
        return;
    }
    bool solved = method.solve(time_limit - init_time_cost, 0);
    auto solve_t = clock();

    double total_time_cost = (((double)solve_t - start_t)/CLOCKS_PER_SEC);

    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << solved
              << " in " << total_time_cost << "s " << std::endl;
    std::cout << "solution validation ? " << method.solutionValidation() << std::endl;

    LargeAgentMAPF_InstanceGenerator<2> generator(deserializer.getAgents(), is_occupied, dim);
    InstanceVisualization(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(),
                                     method.getSolution()
    );

//    InstanceVisualization<AgentType>(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(), {});

}


//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
void loadInstanceAndVisualize(const std::string& file_path) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

    LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

    InstanceVisualization(deserializer.getAgents(),
                          generator.getAllPoses(),
                          deserializer.getInstances(),
                          {},
                          {});

}

//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
void loadInstanceAndPlanningLayeredLAMAPF(const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                          const std::string& file_path,
                                          double time_limit = 30,
                                          bool path_constraint = false,
                                          bool debug_mode = true,
                                          bool visualize = false) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

//    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim, 1e7);
//
//    std::cout << " solvable ?  " << !((generator.getConnectionBetweenNode(7, 89773, 108968)).empty()) << std::endl;


    auto instances = deserializer.getInstances();
    auto agents    = deserializer.getAgents();

    std::vector<std::vector<int> > grid_visit_count_table;

    auto start_t = clock();
    MSTimer mst;
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                    instances,
                    agents,
                    dim, is_occupied);

    auto bi_decompose =
     std::make_shared<MAPFInstanceDecompositionBreakLoop<N, HyperGraphNodeDataRaw<N>, Pose<int, N>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            time_limit);

    
    LAMAPF_Paths layered_paths;
    bool detect_loss_solvability;
    layered_paths = layeredLargeAgentMAPF<N, Pose<int, N>>(bi_decompose->all_levels_,
                                                           mapf_func, //
                                                           grid_visit_count_table,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           time_limit - mst.elapsed()/1e3,
                                                           false);
            
    auto end_t = clock();

    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    if(visualize) {
        LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

        InstanceVisualization(deserializer.getAgents(),
                                         generator.getAllPoses(),
                                         deserializer.getInstances(),
                                         layered_paths,
                                         grid_visit_count_table);
    }
}


// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
bool generateInstance(const std::vector<AgentPtr<N> >& agents,
                      const std::string& file_path,
                      int maximum_sample_count = 1e7) {
    gettimeofday(&tv_pre, &tz);

    // get previous texts as backup
    InstanceDeserializer<N> deserializer_1;
    deserializer_1.loadInstanceFromFile(file_path, dim);
    std::vector<std::string> backup_strs = deserializer_1.getTextString();


    // test whether serialize and deserialize will change agent's behavior
    // even we didn't change their behavior explicitly
    InstanceOrients<N> fake_instances;
    for(int i=0; i<agents.size(); i++) {
        fake_instances.push_back({Pose<int, N>{{0,0},0}, Pose<int, N>{{0,0},0}});
    }
    InstanceSerializer<N> fake_serializer(agents, fake_instances);
    if(!fake_serializer.saveToFile(file_path)) {
        std::cout << "fake_serializer save to path " << file_path << " failed" << std::endl;
        return false;
    }
    InstanceDeserializer<N> fake_deserializer;
    if(!fake_deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return false;
    }
    auto new_agents = fake_deserializer.getAgents();

    // restore backup texts
    fake_serializer.saveStrsToFile(backup_strs, file_path);

    LargeAgentMAPF_InstanceGenerator<N> generator(new_agents, is_occupied, dim, maximum_sample_count);

    // debug: print all agents
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent->serialize() << std::endl;
    }

    const auto& instances_and_path = generator.getNewInstance();
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    std::cout << "find instance ? " << !instances_and_path.empty() << " in " << time_cost << "ms " << std::endl;

    if(instances_and_path.empty()) {
        return false;
    }

    InstanceOrients<2> instances;
    for(int i=0; i<instances_and_path.size(); i++) {
        instances.push_back(instances_and_path[i].first);
    }

    std::vector<LAMAPF_Path> solution;
    for(int i=0; i<instances_and_path.size(); i++) {
        solution.push_back(instances_and_path[i].second);
    }
    InstanceSerializer<N> serializer(new_agents, instances);
    if(serializer.saveToFile(file_path)) {
        std::cout << "save to path " << file_path << " success" << std::endl;
        return true;
    } else {
        std::cout << "save to path " << file_path << " failed" << std::endl;
        return false;
    }
}

// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
void generateInstanceAndPlanning(const std::vector<AgentPtr<N> >& agents,
                                 const std::string& file_path,
                                 const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                 int maximum_sample_count = 1e7,
                                 bool debug_mode = true,
                                 bool visualize = false) {

    //    loadInstanceAndPlanning<AgentType, MethodType>(file_path);
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        loadInstanceAndPlanningLayeredLAMAPF<N>(mapf_func, file_path, 5, false, debug_mode, visualize);
    }
}

template<Dimension N>
void loadInstanceAndDecomposition(const std::string& file_path) {
    InstanceDeserializer<N> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale ";
    for(int i=0; i<N-1; i++) {
        std::cout << dim[i] << "*";
    }
    std::cout << dim[N-1] << std::endl;

    auto instances = deserializer.getInstances();
    auto agents    = deserializer.getAgents();

    MSTimer mst;
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                    instances,
                    agents,
                    dim, is_occupied);

    auto bi_decompose =
     std::make_shared<MAPFInstanceDecompositionBreakLoop<N, HyperGraphNodeDataRaw<N>, Pose<int, N>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            60);


    double time_cost = mst.elapsed()/1e3;
    std::cout << "finish decomposition in " << time_cost << "ms " << std::endl;
//    std::cout << "solution validation ? " << lacbs.solutionValidation() << std::endl;

    // InstanceDecompositionVisualization(decomposer);
}


template<Dimension N>
void generateInstanceAndDecomposition(const std::vector<AgentPtr<N> >& agents,
                                      const std::string& file_path,
                                      const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                      int maximum_sample_count = 1e7,
                                      bool debug_mode = true,
                                      bool visualize = false) {
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        loadInstanceAndDecomposition<N>(file_path);
    }
}



//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
std::vector<std::string> loadInstanceAndCompareLayeredLAMAPF(const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                                             const std::string& func_identifer,
                                                             const std::string& file_path,
                                                             double time_limit = 30,
                                                             bool path_constraint = false,
                                                             int level_of_decomposition = 4,
                                                             bool debug_mode = true) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return {};
    }
    std::cout << " map scale ";
    for(size_t i=0; i<N-1; i++) {
        std::cout << dim[i] << "*";
    }
    std::cout << dim[N-1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;
    return LayeredLAMAPFCompare(deserializer.getInstances(),
                                deserializer.getAgents(),
                                mapf_func,
                                func_identifer,
                                time_limit,
                                path_constraint,
                                level_of_decomposition,
                                debug_mode);
}

template<Dimension N>
std::vector<std::string> LayeredLAMAPFCompare(const InstanceOrients<N>& instances,
                                              const AgentPtrs<N>& agents,
                                              const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                              const std::string& func_identifer,
                                              double time_limit = 30,
                                              bool path_constraint = false,
                                              int level_of_decomposition = 4,
                                              bool debug_mode = true) {
    //    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim, 1e7);
//
//    std::cout << " solvable ?  " << !((generator.getConnectionBetweenNode(7, 89773, 108968)).empty()) << std::endl;

    std::vector<std::vector<int> > grid_visit_count_table_layered;

    memory_recorder.clear();
    sleep(1);
    float base_usage = memory_recorder.getCurrentMemoryUsage();
    auto start_t = clock();
    MSTimer mst;                    
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                    instances,
                    agents,
                    dim, is_occupied);

    auto bi_decompose =
     std::make_shared<MAPFInstanceDecompositionBreakLoop<N, HyperGraphNodeDataRaw<N>, Pose<int, N>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            60);

    
    LAMAPF_Paths layered_paths;
    bool detect_loss_solvability;
    layered_paths = layeredLargeAgentMAPF<N, Pose<int, N>>(bi_decompose->all_levels_,
                                                           mapf_func, //
                                                           grid_visit_count_table_layered,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           60,
                                                           false);
    
                                                           
    double decom_time_cost = mst.elapsed()/1e3;                                                       
    // auto layered_paths = layeredLargeAgentMAPF<N>(instances,
    //                                               agents,
    //                                               dim, is_occupied,
    //                                               mapf_func, //CBS::LargeAgentCBS_func<2, AgentType >,
    //                                               grid_visit_count_table_layered,
    //                                               time_limit, decomposer_ptr,
    //                                               path_constraint,
    //                                               level_of_decomposition,
    //                                               debug_mode);

    auto end_t = clock();
    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - base_usage;

    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << agents.size() << " agents, layered " << func_identifer << " find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan / decom 1 time cost / decom 2 time cost / decom 3 time cost
    std::stringstream ss_layered;
    ss_layered << "LAYERED_"  << func_identifer << " " << agents.size() << " "
               << total_time_cost << " "
               << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
               << !layered_paths.empty() << " " << memory_usage << " "
               << pre_dec->initialize_time_cost_ << " "

               << decom_time_cost;

    memory_recorder.clear();
    sleep(1);
    base_usage = memory_recorder.getCurrentMemoryUsage();
    start_t = clock();
    std::vector<std::vector<int> > grid_visit_count_table_raw;
    auto raw_paths = mapf_func(instances,
                               agents,
                               dim, is_occupied,
                               nullptr,
                               grid_visit_count_table_raw,
                               time_limit,
                               {}, nullptr, {}, {}, {}, nullptr); // default null config for layered MAPF
    end_t = clock();
    sleep(1);
    peak_usage = memory_recorder.getMaximalMemoryUsage();
    memory_usage = peak_usage - base_usage;

    total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << agents.size() << " agents, raw " << func_identifer << " find solution ? " << !raw_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan
    std::stringstream ss_raw;
    ss_raw << "RAW_" << func_identifer << " " << agents.size() << " "
           << total_time_cost << " "
           << getSOC(raw_paths) << " " << getMakeSpan(raw_paths) << " "
           << !raw_paths.empty() << " " << memory_usage;

    std::vector<std::string> retv;
    retv.push_back(ss_layered.str());
    retv.push_back(ss_raw.str());

    return retv;
}

// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
std::vector<std::string> generateInstanceAndCompare(const std::vector<AgentPtr<N> >& agents,
                                                    const std::string& file_path,
                                                    const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                                    const std::string& func_identifer,
                                                    double time_limit = 30,
                                                    bool path_constraint = false,
                                                    int maximum_sample_count = 1e7,
                                                    bool debug_mode = false) {
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        auto retv = loadInstanceAndCompareLayeredLAMAPF<N>(mapf_func,
                                                           func_identifer,
                                                           file_path,
                                                           time_limit,
                                                           path_constraint,
                                                           debug_mode);
        return retv;
    } else {
        return {};
    }
}

void clearFile(const std::string& file_path) {
    std::ofstream os(file_path, std::ios::trunc);
    os.close();
}

void writeStrsToEndOfFile(const std::vector<std::string>& strs, const std::string& file_path) {
    std::ofstream os(file_path, std::ios::app);
    if(!os.is_open()) { return; }
    for(int i=0; i<strs.size(); i++) {
        os << strs[i] << "\n";
    }
    os.close();
}


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



void InstanceVisualization(const std::vector<AgentPtr<2> >& agents,
                           const std::vector<PosePtr<int, 2> >& all_poses,
                           const std::vector<InstanceOrient<2> >& instances,
                           const std::vector<LAMAPF_Path>& solution,
                           const std::vector<std::vector<int> >& grid_visit_count_table = {}) {
    zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    int time_index = 0;


    size_t makespan = getMakeSpan(solution);
    draw_all_instance = false;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        if(draw_full_path) {
            for(int i=0; i<solution.size(); i++)
            {
                const auto& path = solution[i];
                if(path.empty()) { continue; }
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses[path[t]]->pt_,
                            pt2 = all_poses[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio/10), COLOR_TABLE[(i) % 30]);
                }
            }
        } else {
            int i = current_subgraph_id;
            if(!solution.empty()) {
                //for(int i=0; i<solution.size(); i++)
                {
                    const auto &path = solution[i];
                    if (!path.empty()) {
                        for (int t = 0; t < path.size() - 1; t++) {
                            Pointi<2> pt1 = all_poses[path[t]]->pt_,
                                    pt2 = all_poses[path[t + 1]]->pt_;
                            canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio / 10),
                                               COLOR_TABLE[(i) % 30]);
                        }
                    }
                }
            }
        }
        if(draw_all_instance) {
            if(draw_full_path) {
                for (int i=0; i<instances.size(); i++)
                {
                    //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                    const auto &instance = instances[i]; // zoom_ratio/10
                    agents[i]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[i%30]);

                    agents[i]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[i%30]);

                    canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                    canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

                }
            } else {
                const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                agents[current_subgraph_id]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);

                agents[current_subgraph_id]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);

                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

            }
        }
        if(draw_path) {
            if(draw_full_path) {
                for(int i=0; i<solution.size(); i++)
                {
                    const auto &path = solution[i];
                    if (path.empty()) { continue; }
                    Pointi<2> pt;
                    int orient = 0;
                    if (time_index <= path.size() - 1) {
                        pt = all_poses[path[time_index]]->pt_;
                        orient = all_poses[path[time_index]]->orient_;
                    } else {
                        pt = all_poses[path.back()]->pt_;
                        orient = all_poses[path.back()]->orient_;
                    }

                    agents[i]->drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                    canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio / 10));

                }
            } else {
                int i = current_subgraph_id;
                if(!solution.empty()) {
                    const auto &path = solution[i];
                    if (!path.empty()) {
                        Pointi<2> pt;
                        int orient = 0;
                        if (time_index <= path.size() - 1) {
                            pt = all_poses[path[time_index]]->pt_;
                            orient = all_poses[path[time_index]]->orient_;
                        } else {
                            pt = all_poses[path.back()]->pt_;
                            orient = all_poses[path.back()]->orient_;
                        }

                        agents[i]->drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                        canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio / 10));
                    }
                }
            }
        }
        if(draw_visit_grid_table) {
            if(!grid_visit_count_table.empty()) {
                const auto& local_grid_visit_count_table = grid_visit_count_table[current_subgraph_id];
                if(!local_grid_visit_count_table.empty()){
                    Id total_index = getTotalIndexOfSpace<2>(dim);
                    for(int i=0; i<total_index; i++) {
                        Pointi<2> position = IdToPointi<2>(i, dim);
                        int value = local_grid_visit_count_table[i];
                        if(value != 0) {
                            std::stringstream ss;
                            ss << value;
                            canvas.drawTextInt(position[0], position[1], ss.str().c_str(), cv::Vec3b::all(200), .5);
                        }
                    }
                }
            }
        }
        char key = canvas.show(1000);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'w':
                current_subgraph_id ++;
                current_subgraph_id = current_subgraph_id % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 's':
                current_subgraph_id --;
                current_subgraph_id = (current_subgraph_id + instances.size()) % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 'p':
                draw_path = !draw_path;
                break;
            case 'f':
                draw_full_path = !draw_full_path;
                break;
            case 'g':
                draw_visit_grid_table = !draw_visit_grid_table;
                break;
            case 'q':
                if(makespan > 0) {
                    time_index = time_index + makespan - 1;
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            case 'e':
                if(makespan > 0) {
                    time_index++;
//                    if(time_index > makespan) {
//                        time_index = makespan;
//                    }
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            default:
                break;
        }
    }
}

double orientToRadius(const int& orient) {
    double retv = 0;
    switch (orient)
    {
        case 0:
            retv = 0; // 0 degree
            break;
        case 1:
            retv = M_PI; // 180 degree
            break;
        case 2:
            retv = 1.5*M_PI; // 270 degree
            break;
        case 3:
            retv = 0.5*M_PI; // 90 degree
            break;                   
        default:
            std::cout << "ERROR: invalid orient = " << orient << std::endl;
            exit(1);
            break;
    }
    return retv;
}



// maintain all poses of current agents, x, y, z, yaw
extern std::vector<Pointf<3> > allAgentPoses;

extern std::vector<Pointf<3> > allAgentVels;

extern std::pair<AgentPtrs<2>, InstanceOrients<2> > instances;

const double control_frequency = 10;//.033;

double global_offset_x, global_offset_y;

int sub_count_ = 0;

float agent_height = 0.15;

struct CenteralController;

extern CenteralController* ctl;

// assume center of map is (0, 0) in the world coordinate system
Pointf<3> GridToPtf(const Pointi<2>& pt) {
    Pointf<3> retv = {0, 0, 0};
    retv[0] = reso*pt[0] - .5*dim[0]*reso;
    retv[1] = .5*dim[1]*reso - reso*pt[1];
    return retv;
}

Pointf<3> PoseIntToPtf(const PosePtr<int, 2>& pose) {
    Pointf<3> ptf = GridToPtf(pose->pt_);
    ptf[2] = orientToRadius(pose->orient_);
    return ptf;
}

// sdf file path of real robot Circle/Block2D agents in LA-MAPF
std::vector<std::string> ROBOT_SDFS = {
    "/home/yaozhuo/.gz_models_yz/youbot/model.sdf", // ok
    "/home/yaozhuo/.gz_models_yz/atmos/model.sdf", // ok
    "/home/yaozhuo/.gz_models_yz/cleanerbote/model.sdf", // ok
    "/home/yaozhuo/.gz_models_yz/tinyrobot/model.sdf",  // ok
    "/home/yaozhuo/.gz_models_yz/deliveryrobotwithconveyor/model.sdf" // ok
};

// what above robot are used to generate instance
std::vector<int> REAL_ROBOTS = {0,0,0,0,0,0
                                };

// std::vector<int> REAL_ROBOTS = {0,0,0,0,0,0,
//                                 1,1,1,1,1,1,
//                                 //2,2,2,
//                                 3,3,3,3,3,3,
//                                 4,4,4,4,4,4};

// related agent ptrs of above agents. in grid
// AgentPtrs<2> agents = {
//     std::make_shared<BlockAgent_2D >(Pointf<2>({-0.3/reso, -0.2/reso}), Pointf<2>({0.32/reso, 0.2/reso}), 0, dim),
//     std::make_shared<CircleAgent<2>>(.2/reso, 1, dim),
//     std::make_shared<BlockAgent_2D >(Pointf<2>({-0.1/reso, -0.4/reso}), Pointf<2>({1.0/reso, 0.4/reso}), 2, dim),
//     std::make_shared<BlockAgent_2D >(Pointf<2>({-0.4/reso, -0.2/reso}), Pointf<2>({0.1/reso, 0.2/reso}), 3, dim),
//     std::make_shared<BlockAgent_2D >(Pointf<2>({-0.5/reso, -0.3/reso}), Pointf<2>({0.5/reso, 0.3/reso}), 4, dim),

// };

// specify a file state what agents in instance 

#endif //LAYEREDMAPF_COMMON_INTERFACES_H
