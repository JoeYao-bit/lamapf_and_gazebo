#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"





// load map
std::string map_name = pkg_path+"/map/my_map_2602052038_plan";

PictureLoader loader_local(map_name+".pgm", is_grid_occupied_pgm);

std::string yaml_file_path = map_name+".yaml";

DimensionLength* dim_local = loader_local.getDimensionInfo();

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    auto agent_ptr = std::make_shared<CircleAgent<2> >(0.2, 0, dim_local);

    double time_interval = 0.1;// s

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    // 创建局部控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowControllerReal>(MotionConfig());
    auto rot_ctl  = std::make_shared<ConstantRotateController>(MotionConfig());

    // create local control node

    auto agent_control_node = std::make_shared<LocalController>(agent_ptr, line_ctl, 1, time_interval,
                                                                "amcl_pose",
                                                                "local_goal",
                                                                "scan",
                                                                "commands/velocity");  
                                                        

    executor.add_node(agent_control_node);

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();

    return 0;

}