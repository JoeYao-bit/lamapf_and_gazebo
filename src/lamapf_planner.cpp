#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"
#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

CenteralController* ctl = nullptr;

std::vector<Pointf<3> > allAgentPoses;
std::vector<Pointf<3> > allAgentVels;

std::pair<AgentPtrs<2>, InstanceOrients<2> >  instances;

std::vector<LAMAPF_Path> allAgentPaths;

std::shared_ptr<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>> pre_dec = nullptr;

EntityPoseSetterPtr set_pose_clinet = nullptr;

EntitySpawnerPtr entity_pawner_clinet = nullptr;

bool paused = false;
bool gazebo_gui = true;
// class InitExecutionSubscriber : public rclcpp::Node
// {
// public:
//   InitExecutionSubscriber()
//   : Node("path_execution_subscriber")
//   {
//     subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "init_exe", 10, std::bind(&InitExecutionSubscriber::topic_callback, this, _1));
//   }

// private:

//   void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "I heard: '%s', reset controller progress", msg->data.c_str());
//     sub_count_ ++;
//     if(ctl != nullptr) {
//         ctl->clearAllProgress();
//     }
//     for (int id=0; id<instances.first.size(); id++) {
//         const auto& agent = instances.first[id];
//         const auto& start_pt = instances.second[id].first.pt_; 
//         auto color = COLOR_TABLE[id%30];

//         geometry_msgs::msg::Pose initial_pose;

//         Pointf<3> ptf = GridToPtf(start_pt);

//         initial_pose.position.x = ptf[0];  // 设置模型初始位置
//         initial_pose.position.y = ptf[1];
//         initial_pose.position.z = agent_height + sub_count_*0.1;

//         allAgentPoses[id][0] = initial_pose.position.x;
//         allAgentPoses[id][1] = initial_pose.position.y;
//         allAgentPoses[id][2] = ptf[2];
//     }

//   }

//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// set all poses of agent in gazebo to allAgentPoses
void updateAllAgentPoseInGazebo(const EntityPoseSetterPtr& set_pose_clinet_ptr,
                                const rclcpp::Node::SharedPtr& node) {
    assert(allAgentPoses.size() == instances.second.size());
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];

        double target_x = allAgentPoses[id][0],
               target_y = allAgentPoses[id][1],
               target_z = agent_height,
               target_theta = allAgentPoses[id][2];
        if(agent->type_ == "Circle") {

            setModelPose(std::string("Circle_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet_ptr, node);

        } else if(agent->type_ == "Block_2D") {
           
            setModelPose(std::string("Block_2D_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet_ptr, node);

        }
    }
}

int PathVisualize() {
    Canvas canvas("LA-MAPF visualization", dim[0], dim[1], 1./reso, 5);
    canvas.resolution_ = 1./reso;
    assert(pre_dec != nullptr);
    const auto& all_poses = pre_dec->all_poses_;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        // draw all agent's unfinished path
        for(int i=0; i<allAgentPaths.size(); i++)
        {
            const auto& path = allAgentPaths[i];
            if(path.empty()) { continue; }
            for(int t=ctl->progress_of_agents_[i]; t<path.size()-1; t++) {
                ctl;
                Pointi<2> pt1 = all_poses[path[t]]->pt_;
                Pointi<2> pt2 = all_poses[path[t+1]]->pt_;
                canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio/10), COLOR_TABLE[(i) % 30]);
            }
        }
        for(int i=0; i<instances.second.size(); i++)
        {
            //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
            const auto &instance = instances.second[i]; // zoom_ratio/10
            //std::cout << "Agent " << *instances.first[i] << "'s pose "  << allAgentPoses[i] << std::endl;
            //std::cout << "canvas.reso = " << canvas.resolution_ << std::endl;
            //std::cout << "canvas.zoom_ratio = " << canvas.zoom_ratio_ << std::endl;
            double x = allAgentPoses[i][0]/reso, y = allAgentPoses[i][1]/reso, orient = allAgentPoses[i][2];
            instances.first[i]->drawOnCanvas(Pointf<3>{x, y, orient}, canvas, COLOR_TABLE[i%30], false);
            
            //canvas.drawArrowInt(allAgentPoses[i].pt_[0], allAgentPoses[i].pt_[1], -orientToPi_2D(allAgentPoses[i].orient_), 1, std::max(1, zoom_ratio/10));
            //break;
        }
        char key = canvas.show();
        if(key == 32) {
            paused = !paused;
        }
    }
    return 0;
}

void gazeboInitialize(const std::string& file_path, const rclcpp::Node::SharedPtr& node) {
    // 创建服务客户端
    // rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
    // node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // 等待服务可用
    // while (!client->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_INFO(node->get_logger(), "等待 /spawn_entity 服务...");
    // }

    set_pose_clinet = std::make_shared<EntityPoseSetter>();
    entity_pawner_clinet = std::make_shared<EntitySpawner>();


    // rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr get_model_list_clinet =
    //         node->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");

    // // 等待服务可用
    // while (!get_model_list_clinet->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_INFO(node->get_logger(), "等待 /get_model_list 服务...");
    //     rclcpp::spin_some(node);  // 关键：允许节点处理发现消息
    // }

    // // 发送请求
    // auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    // using ServiceResponseFuture = rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedFuture;

    // // auto future = get_model_list_clinet->async_send_request(request, response_received_callback);
    // auto future = get_model_list_clinet->async_send_request(request);
    // // 处理响应
    // std::vector<std::string> delete_model_list;
    // if (rclcpp::spin_until_future_complete(node->shared_from_this(), future) == 
    //     rclcpp::FutureReturnCode::SUCCESS) {
    //     auto response = future.get();
    //     RCLCPP_INFO(node->get_logger(), "当前模型列表:");
    //     for (const auto& name : response->model_names) {
    //         RCLCPP_INFO(node->get_logger(), "%s", name.c_str());
    //         std::string copy_of_line = name;
    //         std::vector<std::string> strs;
    //         boost::split(strs, copy_of_line, boost::is_any_of("_"), boost::token_compress_on);
    //         if(strs[0] == "Circle" || strs[0] == "Block") {
    //             delete_model_list.push_back(name);
    //             RCLCPP_INFO(node->get_logger(), "try delete %s", name.c_str());
    //         }
    //     }
    // } else {
    //     RCLCPP_ERROR(node->get_logger(), "获取模型列表失败");
    // }

    // // delete model of agent
    // rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_model_clinet =
    //         node->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    // // 等待服务可用
    // while (!delete_model_clinet->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_INFO(node->get_logger(), "等待delete_entity服务就绪...");
    // }
    // for(const auto& model_name : delete_model_list) {
    //     // 构造请求
    //     auto request2 = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    //     request2->name = model_name;

    //     // 发送请求
    //     auto future2 = delete_model_clinet->async_send_request(request2);

    //     // 处理响应
    //     if (rclcpp::spin_until_future_complete(node->shared_from_this(), future2) == 
    //         rclcpp::FutureReturnCode::SUCCESS) {
    //         auto response = future2.get();
    //         if (response->success) {
    //             RCLCPP_INFO(node->get_logger(), "模型 %s 删除成功", model_name.c_str());
    //         } else {
    //             RCLCPP_ERROR(node->get_logger(), "模型 %s 删除失败", model_name.c_str());
    //         }
    //     } else {
    //         RCLCPP_ERROR(node->get_logger(), "服务调用失败");
    //     }
    // }

    // add instance to gazebo
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;  // 设置模型初始位置
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.5;
    initial_pose.orientation.w = 1.0;

    // add agent to gazebo
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];
        const auto& start_pt = instances.second[id].first.pt_; 
        auto color = COLOR_TABLE[id%30];
        double start_theta = orientToRadius(instances.second[id].first.orient_);

        Pointf<3> ptf = GridToPtf(start_pt);

        initial_pose.position.x = ptf[0];  // transfer to world coordinate system
        initial_pose.position.y = ptf[1];
        initial_pose.position.z = agent_height;

        allAgentPoses[id][0] = initial_pose.position.x;
        allAgentPoses[id][1] = initial_pose.position.y;
        allAgentPoses[id][2] = start_theta;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, start_theta); // Create this quaternion from roll/pitch/yaw (in radians)

        initial_pose.orientation.x = orientation.x();
        initial_pose.orientation.y = orientation.y();
        initial_pose.orientation.z = orientation.z();
        initial_pose.orientation.w = orientation.w();

        // do not spawn circle or block, but spawn real robots
        std::string file_path3 = ROBOT_SDFS[REAL_ROBOTS[id]];
        if(agent->type_ == "Circle") {
            spawnAgentGazebo(file_path3, std::string("Circle_")+std::to_string(agent->id_), initial_pose, entity_pawner_clinet, node);
        } else if(agent->type_ == "Block_2D") {
            spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, entity_pawner_clinet, node);
        } else {
            RCLCPP_INFO(node->get_logger(), "undefined agent type");
            std::exit(0);
        }
    }
}

void layeredLargeAgentMAPFTest(const std::string& file_path,
                               const rclcpp::Node::SharedPtr& node) {

    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::stringstream ss;
    ss << "map scale = " << dim[0] << "*" << dim[1];
    RCLCPP_INFO(node->get_logger(), ss.str().c_str());

    instances = deserializer.getTestInstance({25}, 1).front(); // get all instances

    RCLCPP_INFO(node->get_logger(), "instance's size %i", instances.first.size());
                            
    allAgentPoses.resize(instances.second.size(), {0,0,0}); // x, y, yaw
    allAgentVels.resize(instances.second.size(), {0,0,0}); // x, y, yaw

    // add agent to gazebo
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];
        const auto& start_pt = instances.second[id].first.pt_; 
        auto color = COLOR_TABLE[id%30];
        double start_theta = orientToRadius(instances.second[id].first.orient_);

        Pointf<3> ptf = GridToPtf(start_pt);

        allAgentPoses[id][0] = ptf[0];
        allAgentPoses[id][1] = ptf[1];
        allAgentPoses[id][2] = start_theta;
    }

    if(gazebo_gui) { gazeboInitialize(file_path, node); }

    std::vector<std::vector<int> > grid_visit_count_table;
    auto start_t = clock();
    MSTimer mst;

    pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>>(
                    instances.second,
                    instances.first,
                    dim, is_occupied);

    auto bl_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, HyperGraphNodeDataRaw<2>, Pose<int, 2>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            60);
 
    bool detect_loss_solvability = false;        
    LAMAPF_Paths layered_paths;
    layered_paths = layeredLargeAgentMAPF<2, Pose<int, 2>>(bl_decompose->all_levels_,
                                                           CBS::LargeAgentCBS_func<2, Pose<int, 2> >, //
                                                           grid_visit_count_table,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           60 - mst.elapsed()/1e3,
                                                           false);        

    auto end_t = clock();
 
    double time_cost = ((double)end_t-start_t)/CLOCKS_PER_SEC;

    std::stringstream ss2;
    ss2 << (layered_paths.size() == instances.first.size() ? "success" : "failed")
              << " layered large agent mapf in " << time_cost << "s " << std::endl;
    RCLCPP_INFO(node->get_logger(), ss2.str().c_str());

    if(layered_paths.empty()) {
        exit(0);
    }
 
    allAgentPaths = layered_paths;

    rclcpp::WallRate loop_rate(control_frequency);

    std::vector<LineFollowControllerPtr> line_ctls(instances.first.size(), std::make_shared<ConstantLineFollowController>(MotionConfig()));
    std::vector<RotateControllerPtr> rot_ctls(instances.first.size(), std::make_shared<ConstantRotateController>(MotionConfig()));

    std::cout << "pre_dec->all_poses_.size() = " << pre_dec->all_poses_.size() << std::endl;

    ctl = new CenteralController(layered_paths, instances.first, pre_dec->all_poses_, line_ctls, rot_ctls, node);


    // InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
    //                        instances.front().second, layered_paths, grid_visit_count_table);

    // start a indenpendent thread to visualize all path
    // std::thread canvas_thread(InstanceVisualization, instances.first, decomposer_ptr->getAllPoses(),
    //                        instances.second, layered_paths, grid_visit_count_table);        

    // canvas_thread.detach();


    // visualize all agent's unfinished path
    std::thread canvas_thread(PathVisualize);
    canvas_thread.detach();

    while (rclcpp::ok())
    {    
        std::cout << "flag 1" << std::endl;
        RCLCPP_INFO(node->get_logger(), "control loop");

        if(!paused) {
            std::cout << "flag 1.1" << std::endl;
            Pointfs<3> new_vels = ctl->calculateCMD(allAgentPoses, allAgentVels, 1/control_frequency);
             std::cout << "flag 2" << std::endl;
            allAgentVels = new_vels;

            for(int i=0; i<instances.first.size(); i++) {
                allAgentPoses[i] = updateAgentPose(allAgentPoses[i], new_vels[i], 1/control_frequency);
                // break;
            }
            std::cout << "flag 3" << std::endl;

        }
        // std::cout << "flag 3" << std::endl;

        if(gazebo_gui) { updateAllAgentPoseInGazebo(set_pose_clinet, node); }
        // std::cout << "flag 4" << std::endl;


        // rclcpp::spin_some(init_exe_sub);
        loop_rate.sleep();
    }

    delete ctl;

    // TODO: set all agent to target



//    gettimeofday(&tv_pre, &tz);
//    CBS::LargeAgentCBS<2, CircleAgent<2> > solver(deserializer.getInstances(), deserializer.getAgents(),
//                                                  dim, is_occupied);
//    gettimeofday(&tv_after, &tz);
//    double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
//    std::vector<LAMAPF_Path> raw_path;
//    if(solver.solve(60)) {
//        raw_path = solver.getSolution();
//    }
//    std::cout << (raw_path.size() == deserializer.getAgents().size() ? "success" : "failed")
//              << " raw large agent mapf in " << time_cost1 << "ms " << std::endl;

    // InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
    //                       instances.front().second, layered_paths, grid_visit_count_table);
}




int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"), node);



    return 0;
}