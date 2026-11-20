#include <cstdio>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"
#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

CenteralControllerFull* ctl = nullptr;

std::vector<Pointf<3> > allAgentPoses;
std::vector<Pointf<3> > allAgentVels;

std::pair<AgentPtrs<2>, InstanceOrients<2> >  instances;

std::vector<LAMAPF_Path> allAgentPaths;
 
std::shared_ptr<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>> pre_dec = nullptr;

EntityPoseSetterPtr set_pose_clinet = nullptr;

EntitySpawnerPtr entity_pawner_clinet = nullptr;

bool paused = true;

int PathVisualize() {
    Canvas canvas("LA-MAPF visualization", dim[0], dim[1], 1./reso, std::min(800./dim[0], 800./dim[1]));
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
            double x = allAgentPoses[i][0], y = allAgentPoses[i][1], orient = allAgentPoses[i][2];
            //std::cout << "x, y, orient = " << x << ", " << y << ", " << orient << std::endl; // ok
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

        Pointf<3> ptf = GridToPtfPicOnly(start_pt);

        allAgentPoses[id][0] = ptf[0];
        allAgentPoses[id][1] = ptf[1];
        allAgentPoses[id][2] = start_theta;
    }

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


    ctl = new CenteralControllerFull(layered_paths, instances.first, pre_dec->all_poses_, line_ctls, rot_ctls, node);


    // visualize all agent's unfinished path
    std::thread canvas_thread(PathVisualize);
    canvas_thread.detach();
    while (rclcpp::ok())
    {    
        RCLCPP_INFO(node->get_logger(), "control loop");
        if(!paused) {
            Pointfs<3> new_vels = ctl->calculateCMD(allAgentPoses, allAgentVels, 1/control_frequency);
            allAgentVels = new_vels;
            for(int i=0; i<instances.first.size(); i++) {
                allAgentPoses[i] = updateAgentPose(allAgentPoses[i], new_vels[i], 1/control_frequency);
                // break;
            }
        }
        loop_rate.sleep();
    }

    delete ctl;

}

int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"), node);



    return 0;
}