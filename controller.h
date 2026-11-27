#ifndef LAMAPF_CONTROLLER
#define LAMAPF_CONTROLLER

#include "common_interfaces.h"
#include "action_dependency_graph.h"

// maintain a action dependency graph,
// do not require each agent's pose to determine whether move to next target
// receive local controller's plan state to determine whether
class CenteralControllerNew {
public:
    CenteralControllerNew(DimensionLength* dim, 
                       IS_OCCUPIED_FUNC<2> is_occupied,
                       const std::pair<AgentPtrs<2>, InstanceOrients<2> >& instances,
                       bool enable_opencv_window = true
                    ) {
        
        dim_ = dim;
        instances_ = instances;                
        isoc_ = is_occupied;
        std::cout << "construct all possible poses" << std::endl;

        Id total_index = getTotalIndexOfSpace<2>(this->dim_);

        all_poses_.resize(total_index * 2 * 2, nullptr); // a position with 2*N orientation

        Pointi<2> pt;

        for (Id id = 0; id < total_index; id++) {
            pt = IdToPointi<2>(id, this->dim_);
            if (!is_occupied(pt)) {
                for (int orient = 0; orient < 2 * 2; orient++) {
                    PosePtr<int, 2> pose_ptr = std::make_shared<Pose<int, 2> >(pt, orient);
                    all_poses_[id * 2 * 2 + orient] = pose_ptr;
                }
            }
        }

        // calculate initial paths 
        auto paths = MAPF(dim, is_occupied, instances);
        
        if(paths.empty()) {
            //RCLCPP_INFO(this->get_logger(), "failed to generate initial paths");
            std::cout << "failed to generate initial paths" << std::endl;
            return;
        }

        assert(paths.size() == instances.first.size());

        auto agents = instances.first;

//        for(int i=0; i<agents.size(); i++) {
//            std::cout << "path " << i << " size = " << paths[i].size() << std::endl;
//        }

        ADG_ = std::make_shared<ActionDependencyGraph<2>>(paths, agents, all_poses_);

        progress_of_agents_.resize(instances.first.size(), 0);
        
        agent_finishes_.resize(instances.first.size(), false);

        for(int i=0; i<instances.first.size(); i++) {
            ADG_->setActionLeave(i, 0);
        }

        // visualize all agent's unfinished path
        if(enable_opencv_window) {
            std::thread canvas_thread(PathVisualize, is_occupied);
            canvas_thread.detach();
        }

    }

    LAMAPF_Paths MAPF(DimensionLength* dim, const IS_OCCUPIED_FUNC<2>& is_occupied,
                      const std::pair<AgentPtrs<2>, InstanceOrients<2> >&  instances) const {

        std::vector<std::vector<int> > grid_visit_count_table;
        auto start_t = clock();
        MSTimer mst;

        std::shared_ptr<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>> pre_dec =
                std::make_shared<PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>>>(
                        instances.second,
                        instances.first,
                        dim, is_occupied);

        auto bl_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<2, HyperGraphNodeDataRaw<2>, 
                                             Pose<int, 2>>>(dim,
                                                            pre_dec->connect_graphs_,
                                                            pre_dec->agent_sub_graphs_,
                                                            pre_dec->heuristic_tables_sat_,
                                                            pre_dec->heuristic_tables_,
                                                            60);
    
        bool detect_loss_solvability = false;        
        LAMAPF_Paths layered_paths;
        layered_paths = layeredLargeAgentMAPF<2, Pose<int, 2>>(bl_decompose->all_levels_,
                                                            LargeAgentLaCAMPose_func<2>,
                                                            //CBS::LargeAgentCBS_func<2, Pose<int, 2> >, //
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
        std::cout << ss2.str() << std::endl;
        return layered_paths;
    }

    // update progress
    void updateADG() {

        Pointf<3> start_ptf, target_ptf;
        bool all_finished = true;
        //RCLCPP_INFO(this->get_logger(), "update ADG");
        for(int i=0; i<ADG_->agents_.size(); i++) {
            //RCLCPP_INFO(this->get_logger(), "flag 0.1");
            bool finished = false;
            // update progress, considering agent may wait at current position multiple times
            while(true) {
                //RCLCPP_INFO(this->get_logger(), "flag 0.2");
                if(progress_of_agents_[i] < ADG_->paths_[i].size()-1) {
                    //RCLCPP_INFO(this->get_logger(), "flag 0.21");
                    //std::cout << "progress_of_agents_.size() = " << progress_of_agents_.size() << std::endl;
                    //std::cout << "progress_of_agents_[i] = " << progress_of_agents_[i] << std::endl;
                    //std::cout << "ADG_->paths_[i].size() = " << ADG_->paths_[i].size() << std::endl;

                    // check whether reach current target, if reach, update progress to next pose
                    if(agent_finishes_[i]) {
                        //RCLCPP_INFO(this->get_logger(), , i);
                        std::cout << "agent_ " << i << " reach temporal target state" << std::endl;
                        // reach current target, move to next target
                        ADG_->setActionLeave(i, progress_of_agents_[i]);
                        // if next action is valid
                        if(!ADG_->isActionValid(i, progress_of_agents_[i] + 1)) {
                            //RCLCPP_INFO(this->get_logger(), "agent_%i next action %i invalid", i, progress_of_agents_[i] + 1);
                            std::cout << "agent_ " << i<< " next action %i invalid" << std::endl;
                            break;
                        }
                        std::cout << "agent_" << i << "'s progress update from "
                                  << progress_of_agents_[i] << "/" << ADG_->paths_[i].size() << " to "
                                  << progress_of_agents_[i]+1 <<  "/" << ADG_->paths_[i].size() << std::endl;

                        progress_of_agents_[i]++;
                        agent_finishes_[i] = false;
                    } else {
                        //RCLCPP_INFO(this->get_logger(), "flag 1.1");
                        break;
                    }
                } else {
                    //RCLCPP_INFO(this->get_logger(), "agent_%i 's task finish", i);
                    std::cout << "agent_" << i <<  "'s task finish" << std::endl;
                    // when finish tell it to wait at target
                    finished = true;
                    break;
                }
            }
            if(finished) { 
                continue;
            } else {
                all_finished = false;
            }
        }
        if(all_finished) {
            std::cout << "all agents reach target" << std::endl;
        }
        return;
    }

    void clearAllProgress() {
        ADG_->clearAllProgress();
        progress_of_agents_.resize(ADG_->agents_.size(), 0);
    }
 
    static int PathVisualize(IS_OCCUPIED_FUNC<2> is_occupied) {
        float zoom_ratio = std::max(1., ceil(std::min(800./dim_[0], 800./dim_[1]))); 
        Canvas canvas("LA-MAPF visualization", dim_[0], dim_[1], 1./reso, zoom_ratio);
        std::cout << "canvas rows / cols = " << canvas.getCanvas().rows << " / " << canvas.getCanvas().cols << std::endl;
        std::cout << "dim_[0] = " << dim_[0] << ", dim_[1] = " << dim_[1] << std::endl;
        std::cout << "before zoom_ratio = " << zoom_ratio << ", canvas.resolution_ = " << canvas.resolution_ << std::endl;
        canvas.resolution_ = 1./reso;
        std::cout << "after zoom_ratio = " << zoom_ratio << ", canvas.resolution_ = " << canvas.resolution_ << std::endl;
        while(true) {
            canvas.resetCanvas();
            canvas.drawGridMap(dim_, is_occupied);
            for(int i=0; i<ADG_->paths_.size(); i++)
            {
                const auto& path = ADG_->paths_[i];
                if(path.empty()) { continue; }
                for(int t=progress_of_agents_[i]; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses_[path[t]]->pt_;
                    Pointi<2> pt2 = all_poses_[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1],
                                       true,
                                       std::max(1., zoom_ratio/10.),
                                       COLOR_TABLE[(i) % 30]);
                }
            }
            // draw every agent's pose
            for(int i=0; i<instances_.second.size(); i++)
            {
                const auto &instance = instances_.second[i]; // zoom_ratio/10
                size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
                PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
                instances_.first[i]->drawOnCanvas(*start_pose, canvas, cv::Vec3b::all(0), true);
            }
            char key = canvas.show(100);
            if(key == 32) {
                paused = !paused;
            }
        }
        return 0;
    }


    static DimensionLength* dim_;

    IS_OCCUPIED_FUNC<2> isoc_;

    static std::pair<AgentPtrs<2>, InstanceOrients<2> > instances_;

    static std::vector<std::shared_ptr<Pose<int, 2>> > all_poses_;

    static std::shared_ptr<ActionDependencyGraph<2> > ADG_;

    static Pointfs<3> all_agent_poses_;

    static std::vector<int> progress_of_agents_;

    static std::vector<bool> agent_finishes_;

};



#endif





