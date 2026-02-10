#ifndef LAMAPF_CONTROLLER
#define LAMAPF_CONTROLLER

#include "common_interfaces.h"
#include "action_dependency_graph.h"
#include "local_controller.h"


#include "lamapf_and_gazebo/fake_agents.h"
#include "sensor_msgs/msg/laser_scan.hpp"


class CenteralControllerFull {
public:
    CenteralControllerFull(const std::vector<LAMAPF_Path>& paths,
                       const std::vector<AgentPtr<2>>& agents,
                       const std::vector<PosePtr<int, 2> >& all_poses,
                       const std::vector<LineFollowControllerPtr>& line_ctls,
                       const std::vector<RotateControllerPtr>& rot_ctls,
                       const rclcpp::Node::SharedPtr& node_ptr): 
                       ADG_(paths, agents, all_poses),
                       line_ctls_(line_ctls),
                       rot_ctls_(rot_ctls),
                       node_ptr_(node_ptr) {

        progress_of_agents_.resize(agents.size(), 0);

        assert(paths.size() == agents.size());
        assert(line_ctls.size() == agents.size());
        assert(rot_ctls.size() == agents.size());

    }

    // calculate vel cmd for each agent
    Pointfs<3> calculateCMD(const Pointfs<3>& poses, const Pointfs<3>& vels, const float& time_interval) {
        Pointfs<3> retv(poses.size(), {0, 0, 0});
        Pointf<3> cmd_vel;
        Pointf<3> start_ptf, target_ptf;
        bool all_finished = true;
        //RCLCPP_INFO(node_ptr_->get_logger(), "flag 0");
        for(int i=0; i<ADG_.agents_.size(); i++) {
            //RCLCPP_INFO(node_ptr_->get_logger(), "flag 0.1");
            cmd_vel = Pointf<3>{0, 0, 0};
            Pointf<3> cur_pose = poses[i];
            size_t target_pose_id;
            PosePtr<int, 2> target_pose;
            bool finished = false;
            // update progress, considering agent may wait at current position multiple times
            while(true) {
                //RCLCPP_INFO(node_ptr_->get_logger(), "flag 0.2");
                if(progress_of_agents_[i] < ADG_.paths_[i].size()-1) {
                    target_pose_id = ADG_.paths_[i][progress_of_agents_[i] + 1];

                    //RCLCPP_INFO(node_ptr_->get_logger(), "flag 0.3");

                    //std::cout << "target_pose_id = " << target_pose_id << ", ADG_.all_poses_.size = " << ADG_.all_poses_.size() << std::endl;

                    target_pose = ADG_.all_poses_[target_pose_id];
                    target_ptf = PoseIntToPtf(target_pose, GridToPtfPicOnly);
                    
                    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();
                    float angle_to_target = fmod(target_ptf[2]-cur_pose[2], 2*M_PI);
                    // std::cout << "dist_to_target = " << dist_to_target << " / angle_to_target " << angle_to_target << std::endl;
                    size_t start_pose_id = ADG_.paths_[i][progress_of_agents_[i]];;
                    //RCLCPP_INFO(node_ptr_->get_logger(), "start_pose_id / target_pose_id = %i / %i", start_pose_id, target_pose_id);
                    //std::stringstream ss;
                    //ss << "flag 0.4" << ", dist_to_target = " << dist_to_target << " / angle_to_target " << angle_to_target;
                    //RCLCPP_INFO(node_ptr_->get_logger(), ss.str());
                    //std::stringstream ss2;
                    //ss2 << "flag 0.5" << ", cur pose " << cur_pose <<  "/ start pose " << *ADG_.all_poses_[start_pose_id] << " / target pose = " << *target_pose;
                    // check whether reach current target, if reach, update progress to next pose
                    //RCLCPP_INFO(node_ptr_->get_logger(), ss2.str());
                    if(fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001) {
                        // reach current target, move to next target
                        ADG_.setActionLeave(i, progress_of_agents_[i]);
                        progress_of_agents_[i]++;
                        //std::cout << "agent_" << i  << " at pose " << poses[i] << " reach " << target_ptf << std::endl; 
                        //std::stringstream ss;
                        //ss << "flag 1.0" << ", agent_" << i  << " at pose " << poses[i] << " reach " << target_ptf;
                        //RCLCPP_INFO(node_ptr_->get_logger(), ss.str());
                        // ADG_.setActionProcessing(i, progress_of_agents_[i]);
                    } else {
                        //RCLCPP_INFO(node_ptr_->get_logger(), "flag 1.1");
                        break;
                    }
                } else {
                    //RCLCPP_INFO(node_ptr_->get_logger(), "flag 2");
                    finished = true;
                    target_pose_id = ADG_.paths_[i].back();
                    target_pose = ADG_.all_poses_[target_pose_id];
                    target_ptf = PoseIntToPtf(target_pose, GridToPtfPicOnly);
                    //std::cout << "agent_" << i << " finish all pose, now at " << poses[i] << ", final pose = " << target_ptf << std::endl;
                    break;
                }
            }
            if(finished) { 
                continue;
            } else {
                all_finished = false;
            }
            //RCLCPP_INFO(node_ptr_->get_logger(), "flag 3");
            if(progress_of_agents_[i] < ADG_.paths_[i].size()-1) {
                // check whether next move valid in ADG, if not valid, wait till valid
                if(!ADG_.isActionValid(i, progress_of_agents_[i])) {
                    continue;
                }

                // if not finish all pose, get cmd move to next pose, otherwise stop 
                size_t start_pose_id = ADG_.paths_[i][progress_of_agents_[i]];;
                PosePtr<int, 2> start_pose = ADG_.all_poses_[start_pose_id];
                start_ptf = PoseIntToPtf(start_pose, GridToPtfPicOnly);


                // check whether need wait, via ADG
                retv[i] = Pointf<3>{0, 0, 0}; 
                //RCLCPP_INFO(node_ptr_->get_logger(), "progress_of_agents_ / paht length = %i / %i", progress_of_agents_[i], ADG_.paths_[i].size());
                //RCLCPP_INFO(node_ptr_->get_logger(), "start_pose_id / target_pose_id = %i / %i", start_pose_id, target_pose_id);
                // if move to next pose, current pose and next pose must be different 
                assert(start_pose_id != target_pose_id); // yz:250412, what if the target repeated in path's end ?

                if(start_pose->pt_ == target_pose->pt_) {
                    // rotate
                    auto& rot_ctr = rot_ctls_[i];
                    rot_ctr->ang_ = target_ptf[2]; // update target angle

                    if(fabs(target_ptf[2] - start_ptf[2]) <= 0.5*M_PI + 0.001) {
                        if(target_ptf[2] > start_ptf[2]) { rot_ctr->posi_rot_ = true; }
                        else { rot_ctr->posi_rot_ = false; }
                    } else {
                        if(target_ptf[2] > start_ptf[2]) { rot_ctr->posi_rot_ = false; }
                        else { rot_ctr->posi_rot_ = true; }                    
                    }
                    //std::stringstream ss;
                    //ss << "current pose " << poses[i] << "start pose = " << start_ptf << ", target dir/ang = " << rot_ctr->posi_rot_ << ", " << rot_ctr->ang_;
                    //RCLCPP_INFO(node_ptr_->get_logger(), ss.str());
                    
                    cmd_vel = rot_ctr->calculateCMD(poses[i], vels[i], time_interval);

                } else {
                    // move forward
                    auto& line_ctr = line_ctls_[i];
                    line_ctr->pt1_ = Pointf<2>({start_ptf[0],  start_ptf[1]});
                    line_ctr->pt2_ = Pointf<2>({target_ptf[0], target_ptf[1]}); // update target line
                    //std::cout << "current pose " << poses[i] << ", target line = " << line_ctr->pt1_ << ", " << line_ctr->pt2_ << std::endl;
                    cmd_vel = line_ctr->calculateCMD(poses[i], vels[i], time_interval);
                }
                retv[i] = cmd_vel; 
            } else {
                target_pose_id = ADG_.paths_[i].back();
                // std::cout << "target_pose_id = " << target_pose_id << ", ADG_.all_poses_.size = " << ADG_.all_poses_.size() << std::endl;
                target_pose = ADG_.all_poses_[target_pose_id];
                target_ptf = PoseIntToPtf(target_pose, GridToPtfPicOnly);
                //std::cout << "agent_" << i << " finish2, at " << poses[i] << ", target = " << target_ptf << std::endl;
            }
            //std::cout << "cur vel = " << cmd_vel << std::endl;
            // break;
        }
        if(all_finished) {
            RCLCPP_INFO(node_ptr_->get_logger(), "all agent reach target");
        }
        return retv;
    }

    void clearAllProgress() {
        ADG_.clearAllProgress();
        progress_of_agents_.resize(ADG_.agents_.size(), 0);
    }

    ActionDependencyGraph<2> ADG_;

    std::vector<LineFollowControllerPtr> line_ctls_;
    std::vector<RotateControllerPtr> rot_ctls_;

    std::vector<int> progress_of_agents_;

    rclcpp::Node::SharedPtr node_ptr_; // for print log

};



// maintain a action dependency graph, 
// tell single robots when to move, move to where and when stop
// using agent's pose to determine whether finish current task
// there are more than one place determine whether the agent reach target
// so when they are check at different time and localization is shaking (ususally in real robot)
// some part think reach target and other think not
// this cause error 
class CenteralController : public rclcpp::Node  {
public:
    CenteralController(DimensionLength* dim, 
                       IS_OCCUPIED_FUNC<2> is_occupied,
                       const std::pair<AgentPtrs<2>, InstanceOrients<2> >& instances,
                       const POSE_TO_PTF_FUNC& pose_to_ptf_func,
                       const float& time_interval = 0.1,
                       bool enable_opencv_window = true
                       //const GRID_TO_PTF_FUNC& grid_to_ptf_func = GridToPtfPicOnly
                    ): 
                       rclcpp::Node("central_controller"),
                       pose_to_ptf_func_(pose_to_ptf_func) {        
                        
        
        dim_ = dim;
        instances_ = instances;                
        paused_ = false;
        pub_init_target_ = true;
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
            RCLCPP_INFO(this->get_logger(), "failed to generate initial paths");
            return;
        }

        assert(paths.size() == instances.first.size());

        auto agents = instances.first;

        for(int i=0; i<agents.size(); i++) {
            std::cout << "path " << i << " size = " << paths[i].size() << std::endl;
        }

        ADG_ = std::make_shared<ActionDependencyGraph<2>>(paths, agents, all_poses_);

        progress_of_agents_.resize(instances.first.size(), 0);

        all_agent_poses_.resize(instances.first.size());

        for(int i=0; i<instances.first.size(); i++) {
            size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
            Pointf<3> start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);
            all_agent_poses_[i] = start_ptf;

            ADG_->setActionLeave(i, 0);

        }

        // RCLCPP_INFO(this->get_logger(), "flag 1");
        // NOTICE: cache queue size must be large enough to cache all agent's pose or goal
        // otherwise some msg will be ignored

        for(int i=0; i<instances.first.size(); i++) {
            std::stringstream ss3;
            ss3 << "/robot" << i << "/local_goal";
            goal_publishers_.push_back(
                this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>(ss3.str().c_str(), 2*instances.first.size()));
        }

        pose_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>(
                "PoseUpdate", 2*instances.first.size(),
                [this](lamapf_and_gazebo_msgs::msg::UpdatePose::SharedPtr msg) {

                    all_agent_poses_[msg->agent_id][0] = msg->x;
                    all_agent_poses_[msg->agent_id][1] = msg->y;
                    all_agent_poses_[msg->agent_id][2] = msg->yaw;

                    std::stringstream ss;
                    ss << "during CentralController loop, receive pose of agent_" << msg->agent_id;
                    ss << " = " << all_agent_poses_[msg->agent_id];
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                });

         error_state_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::ErrorState>(
                "AgentErrorState", 2*instances.first.size(),
                [this](lamapf_and_gazebo_msgs::msg::ErrorState::SharedPtr msg) {
                    std::stringstream ss;
                    ss << "receive error state from agent_" << msg->agent_id << 
                    ", error state = " << msg->error_state << ", all action paused";
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                    need_replan_ = true;
                    // tell all agent to wait
                    for(int i=0; i<instances_.first.size(); i++) {
                        lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                        msg.agent_id   = i;
                        msg.wait       = true;
                        
                        goal_publishers_[i]->publish(msg); 
                    }
                });               
        
        //sleep(1);  // wait to ensure all local agent will receive initial goal      

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this]() {
            //std::stringstream ss;
            //ss << "during CentralController loop, paused = " << paused_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            if(need_replan_) {
                need_replan_ = false;
                paused_ = true;
                // update map (TODO) and replan plath
                reupdate(all_agent_poses_);
                // tell all agent to stop
                recover_tasks_.clear();
                RCLCPP_INFO(this->get_logger(), "after finish replan, pub all agent's recover task"); 
                for(int i=0; i<instances_.first.size(); i++) {
                    Pose<int, 2> pre_pose = *(ADG_->all_poses_[ADG_->paths_[i][progress_of_agents_[i]]]);
                    Pointf<3> pre_ptf = pose_to_ptf_func_(pre_pose);//PoseIntToPtf(pre_pose, grid_to_ptf_func_);
                    recover_tasks_.push_back(std::make_pair(this->all_agent_poses_[i], pre_ptf));

                    // tell all agents to return to last valid state
                    lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                    msg.start_x   = this->all_agent_poses_[i][0];
                    msg.start_y   = this->all_agent_poses_[i][1];
                    msg.start_yaw = this->all_agent_poses_[i][2];

                    msg.target_x   = pre_ptf[0];
                    msg.target_y   = pre_ptf[1];
                    msg.target_yaw = pre_ptf[2];
                    
                    msg.agent_id   = i;
                    msg.wait       = false;
                    
                    goal_publishers_[i]->publish(msg); 
                }
                paused_ = false;
            }
            if(!paused_) {
                if(recover_tasks_.empty()) {
                    updateADG(all_agent_poses_); 
                } else {
                    RCLCPP_INFO(this->get_logger(), "do recover task");
                    // checck whether finish recover task, 
                    // if finish, clear recover_tasks_
                    bool all_recover_task_finish = true;
                    for(int i=0; i<instances_.first.size(); i++) {
                        if(!reachTarget(all_agent_poses_[i], recover_tasks_[i].second)) {
                            all_recover_task_finish = false;
                            break; 
                        } else {
                            std::stringstream ss;
                            ss << "agent_" << i << "'s recover task finish";
                            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                        }
                    }
                    if(all_recover_task_finish) {
                        recover_tasks_.clear();
                        std::stringstream ss;
                        ss << "all recover task finish";
                        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                    }
                }
            }
        });

        // visualize all agent's unfinished path
        if(enable_opencv_window) {
            std::thread canvas_thread(PathVisualize, is_occupied);
            canvas_thread.detach();
        }

    }

    void pubInitialGoals() {
        // pub all agent's first goal
        for(int i=0; i<instances_.first.size(); i++) {
            if(ADG_->paths_[i].size() < 2) {
                continue;
            }
            std::stringstream ss;

            size_t start_pose_id = ADG_->paths_[i][0];

            ss.str("");ss.clear();
            ss << "agent_" << i << ", start_pose_id = " << start_pose_id;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
            assert(start_pose != nullptr);
            Pointf<3> start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);

            size_t target_pose_id = ADG_->paths_[i][1];
            
            ss.str("");ss.clear();
            ss << "agent_" << i << ", target_pose_id = " << target_pose_id;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            PosePtr<int, 2> target_pose = ADG_->all_poses_[target_pose_id];
            assert(target_pose != nullptr);
            Pointf<3> target_ptf = pose_to_ptf_func_(*target_pose);//PoseIntToPtf(target_pose, grid_to_ptf_func_);

            lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
            msg.start_x   = start_ptf[0];
            msg.start_y   = start_ptf[1];
            msg.start_yaw = start_ptf[2];

            msg.target_x   = target_ptf[0];
            msg.target_y   = target_ptf[1];
            msg.target_yaw = target_ptf[2];
            
            msg.agent_id   = i;
            msg.wait       = false;
            
            goal_publishers_[i]->publish(msg);

            ss.str("");ss.clear();
            ss << "central controller pub agent_" << i << "'s initial goal " << start_ptf <<"->" << target_ptf;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        }
    }

    // TODO: when single agent failed to complete its task, update all agent's path and search path again
    bool reupdate(const Pointfs<3>& poses) {
        std::stringstream ss;
        ss << "start replan of path";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        // get current state as start state of planning
        assert(poses.size() == instances_.first.size());
        Pose<int, 2> pose;
        for(int i=0; i<instances_.first.size(); i++) {
            Pose<int, 2> pre_pose = *(ADG_->all_poses_[ADG_->paths_[i][progress_of_agents_[i]]]);
            instances_.second[i].first = pre_pose;//PtfToPoseInt(poses[i]);
            std::stringstream ss;
            ss << "Agent " << i << "'s initial ptf: " << poses[i] << ", pose " << instances_.second[i].first;
            ss << "| pose to ptf " << pose_to_ptf_func_(instances_.second[i].first);//PoseIntToPtf(instances_.second[i].first, grid_to_ptf_func_);
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }
        // calculate initial paths 
        auto paths = MAPF(dim, is_occupied, instances_);
        
        if(paths.empty()) {
            RCLCPP_INFO(this->get_logger(), "failed to generate initial paths");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "flag 1");
        assert(paths.size() == instances_.first.size());

        auto agents = instances_.first;

        for(int i=0; i<agents.size(); i++) {
            std::cout << "path " << i << " size = " << paths[i].size() << ": ";
            for(int j=0; j<paths[i].size(); j++) {
                std::cout << paths[i][j] << " ";
            }
            std::cout << std::endl;
        }

        ADG_ = std::make_shared<ActionDependencyGraph<2>>(paths, agents, all_poses_);
        RCLCPP_INFO(this->get_logger(), "flag 2");
        progress_of_agents_.clear();
        progress_of_agents_.resize(instances_.first.size(), 0);

        all_agent_poses_.resize(instances_.first.size());
        RCLCPP_INFO(this->get_logger(), "flag 3");
        for(int i=0; i<instances_.first.size(); i++) {
            size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
            Pointf<3> start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);
            all_agent_poses_[i] = start_ptf;

            ADG_->setActionLeave(i, 0);

        }                                                                                                             
        RCLCPP_INFO(this->get_logger(), "finish reupdate paths");

        RCLCPP_INFO(this->get_logger(), "finish pub inital goal after reupdate paths");    
        
        pub_init_target_ = true;

        return true;
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
        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
        return layered_paths;
    }

    // update progress
    void updateADG(const Pointfs<3>& poses) {

        if(pub_init_target_) {
            pubInitialGoals();
            pub_init_target_ = false;// only pub the first time
        }

        Pointf<3> start_ptf, target_ptf;
        bool all_finished = true;
        //RCLCPP_INFO(this->get_logger(), "update ADG");
        for(int i=0; i<ADG_->agents_.size(); i++) {
            //RCLCPP_INFO(this->get_logger(), "flag 0.1");
            Pointf<3> cur_pose = poses[i];
            size_t target_pose_id;
            PosePtr<int, 2> target_pose;
            bool finished = false;
            // update progress, considering agent may wait at current position multiple times
            while(true) {
                //RCLCPP_INFO(this->get_logger(), "flag 0.2");
                if(progress_of_agents_[i] < ADG_->paths_[i].size()-1) {
                    //RCLCPP_INFO(this->get_logger(), "flag 0.21");
                    //std::cout << "progress_of_agents_.size() = " << progress_of_agents_.size() << std::endl;
                    //std::cout << "progress_of_agents_[i] = " << progress_of_agents_[i] << std::endl;
                    //std::cout << "ADG_->paths_[i].size() = " << ADG_->paths_[i].size() << std::endl;
                    target_pose_id = ADG_->paths_[i][progress_of_agents_[i] + 1];
                    //RCLCPP_INFO(this->get_logger(), "flag 0.3");
                    target_pose = ADG_->all_poses_[target_pose_id];
                    target_ptf = pose_to_ptf_func_(*target_pose);//PoseIntToPtf(target_pose, grid_to_ptf_func_);
                    
                    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - 
                                                    Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();

                    float angle_to_target = shortestAngularDistance(target_ptf[2], cur_pose[2]);

                    std::stringstream ss;        
                    ss << "agent_"<< i << ", dist_to_target = " << dist_to_target << ", angle_to_target = " << angle_to_target << ", target = " << target_ptf << ", cur pose = " << cur_pose;
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

                    // check whether reach current target, if reach, update progress to next pose
                    if(reachTarget(cur_pose, target_ptf)) {
                        RCLCPP_INFO(this->get_logger(), "agent_%i reach temporal target state", i);
                        // reach current target, move to next target
                        ADG_->setActionLeave(i, progress_of_agents_[i]);
                        // if next action is valid
                        if(!ADG_->isActionValid(i, progress_of_agents_[i] + 1)) {
                            RCLCPP_INFO(this->get_logger(), "agent_%i next action %i invalid", i, progress_of_agents_[i] + 1);
                            // tell the agent to wait utill action are valid
                            lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                            msg.agent_id   = i;
                            msg.wait       = true;
                            
                            goal_publishers_[i]->publish(msg); 
                            break;
                        }
                        RCLCPP_INFO(this->get_logger(), "agent_%i's progress update from % i/%i to %i/%i",
                                     i, progress_of_agents_[i], ADG_->paths_[i].size(),
                                     progress_of_agents_[i]+1, ADG_->paths_[i].size());
                        progress_of_agents_[i]++;
                        //RCLCPP_INFO(this->get_logger(), "flag 0.4");
                        if(progress_of_agents_[i] + 1 <= ADG_->paths_[i].size() - 1) {
                            //RCLCPP_INFO(this->get_logger(), "flag 0.5");
                            //RCLCPP_INFO(this->get_logger(), ss.str());
                            // update the agent's start and target state
                            size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
                            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
                            start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);

                            size_t target_pose_id = ADG_->paths_[i][progress_of_agents_[i] + 1];
                            PosePtr<int, 2> target_pose = ADG_->all_poses_[target_pose_id];
                            target_ptf = pose_to_ptf_func_(*target_pose);//PoseIntToPtf(target_pose, grid_to_ptf_func_);

                            lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                            msg.start_x   = start_ptf[0];
                            msg.start_y   = start_ptf[1];
                            msg.start_yaw = start_ptf[2];

                            msg.target_x   = target_ptf[0];
                            msg.target_y   = target_ptf[1];
                            msg.target_yaw = target_ptf[2];

                            msg.agent_id   = i;
                            msg.wait       = false;
                            
                            goal_publishers_[i]->publish(msg); 

                            std::stringstream ss;
                            ss << "central controller pub agent_" << i << "'s goal " << start_ptf <<"->" << target_ptf;
                            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

                        }
                    } else {
                        //RCLCPP_INFO(this->get_logger(), "flag 1.1");
                        break;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "agent_%i 's task finish", i);
                    // when finish tell it to wait at target
                    lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                    msg.agent_id   = i;
                    msg.wait       = true;
                            
                    goal_publishers_[i]->publish(msg); 
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
            RCLCPP_INFO(this->get_logger(), "all agents reach target");
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
        while(rclcpp::ok()) {
            canvas.resetCanvas();
            canvas.drawEmptyGrid();
            canvas.drawGridMap(dim_, is_occupied);
            // draw all agent's unfinished path
            for(int i=0; i<ADG_->paths_.size(); i++)
            {
                const auto& path = ADG_->paths_[i];
                if(path.empty()) { continue; }
                for(int t=progress_of_agents_[i]; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses_[path[t]]->pt_;
                    Pointi<2> pt2 = all_poses_[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1., zoom_ratio/10.), COLOR_TABLE[(i) % 30]);
                }
            }
            for(int i=0; i<instances_.second.size(); i++)
            {
                //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                const auto &instance = instances_.second[i]; // zoom_ratio/10
                //std::cout << "Agent " << *instances.first[i] << "'s pose "  << allAgentPoses[i] << std::endl;
                //std::cout << "canvas.reso = " << canvas.resolution_ << std::endl;
                //std::cout << "canvas.zoom_ratio = " << canvas.zoom_ratio_ << std::endl;
                double x = all_agent_poses_[i][0], y = all_agent_poses_[i][1], orient = all_agent_poses_[i][2];

                
                instances_.first[i]->drawOnCanvas(Pointf<3>{x, y, orient}, canvas, COLOR_TABLE[i%30], false);
                
                //canvas.drawArrowInt(allAgentPoses[i].pt_[0], allAgentPoses[i].pt_[1], -orientToPi_2D(allAgentPoses[i].orient_), 1, std::max(1, zoom_ratio/10));
                //break;
            }
            char key = canvas.show();
            if(key == 32) {
                paused_ = !paused_;
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

    // task about all agent return to previous valid state
    // ready for replan
    // used only when task is interrupted
    // empty means current no recover task, otherwise current have recover task
    // when have recover task, do it first
    static std::vector<std::pair<Pointf<3>, Pointf<3> > > recover_tasks_;

    std::vector<rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr> goal_publishers_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_subscriber_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::ErrorState>::SharedPtr error_state_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    static bool paused_;

    static bool pub_init_target_;

    static bool need_replan_;

    //GRID_TO_PTF_FUNC grid_to_ptf_func_;

    POSE_TO_PTF_FUNC pose_to_ptf_func_;

};

// maintain a action dependency graph, 
// do not require each agent's pose to determine whether move to next target
// receive local controller's plan state to determine whether
class CenteralControllerNew : public rclcpp::Node  {
public:
    CenteralControllerNew(DimensionLength* dim, 
                       IS_OCCUPIED_FUNC<2> is_occupied,
                       const std::pair<AgentPtrs<2>, InstanceOrients<2> >& instances,
                       const POSE_TO_PTF_FUNC& pose_to_ptf_func,
                       const float& time_interval = 0.1,
                       bool enable_opencv_window = true
                       //const GRID_TO_PTF_FUNC& grid_to_ptf_func = GridToPtfPicOnly
                    ): 
                       rclcpp::Node("central_controller") {        
                        
        
        dim_ = dim;
        instances_ = instances;                
        paused_ = false;
        pub_init_target_ = true;
        pose_to_ptf_func_ = pose_to_ptf_func;
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
            RCLCPP_INFO(this->get_logger(), "failed to generate initial paths");
            return;
        }

        assert(paths.size() == instances.first.size());

        auto agents = instances.first;

        for(int i=0; i<agents.size(); i++) {
            std::cout << "path " << i << " size = " << paths[i].size() << std::endl;
        }

        ADG_ = std::make_shared<ActionDependencyGraph<2>>(paths, agents, all_poses_);

        progress_of_agents_.resize(instances.first.size(), 0);
        
        agent_finishes_.resize(instances.first.size(), false);

        all_agent_poses_.resize(instances.first.size());

        for(int i=0; i<instances.first.size(); i++) {
            
            size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
            Pointf<3> start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);
            all_agent_poses_[i] = start_ptf;

            ADG_->setActionLeave(i, 0);
        }

        // RCLCPP_INFO(this->get_logger(), "flag 1");
        // NOTICE: cache queue size must be large enough to cache all agent's pose or goal
        // otherwise some msg will be ignored


        for(int i=0; i<instances.first.size(); i++) {
            std::stringstream ss3;
            ss3 << "/robot" << i << "/local_goal";

            // goal_publishers_.push_back(
            //     this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>(ss3.str().c_str(), 2*instances.first.size()));

            auto client = this->create_client<lamapf_and_gazebo_msgs::srv::LocalGoal>(ss3.str());

            RCLCPP_INFO(this->get_logger(), "Waiting for local_goal service...");
            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
            }

            goal_pub_clients_.push_back(client);
        }

        pose_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>(
            "PoseUpdate", 2*instances.first.size(),
            [this](lamapf_and_gazebo_msgs::msg::UpdatePose::SharedPtr msg) {

                all_agent_poses_[msg->agent_id][0] = msg->x;
                all_agent_poses_[msg->agent_id][1] = msg->y;
                all_agent_poses_[msg->agent_id][2] = msg->yaw;

                std::stringstream ss;
                ss << "during CentralController loop, receive pose of agent_" << msg->agent_id;
                ss << " = " << all_agent_poses_[msg->agent_id];
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            });

        error_state_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::ErrorState>(
                "AgentErrorState", 2*instances.first.size(),
                [this](lamapf_and_gazebo_msgs::msg::ErrorState::SharedPtr msg) {
                    std::stringstream ss;
                    ss << "receive error state from agent_" << msg->agent_id << 
                    ", error state = " << msg->error_state << ", all action paused";
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                    need_replan_ = true;
                    // tell all agent to wait
                    for(int i=0; i<instances_.first.size(); i++) {

                        // lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                        // msg.agent_id   = i;
                        // msg.wait       = true;
                        
                        // goal_publishers_[i]->publish(msg); 

                        auto request = std::make_shared<lamapf_and_gazebo_msgs::srv::LocalGoal::Request>();

                        request->wait = true;

                        auto future = goal_pub_clients_[i]->async_send_request(
                            request,
                            std::bind(&CenteralControllerNew::responseCallback, this, std::placeholders::_1)
                        );
                    }
                });               
        
        agent_state_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::AgentState>(
                "AgentState", 2*instances.first.size(),
                [this](lamapf_and_gazebo_msgs::msg::AgentState::SharedPtr msg) {
                    std::stringstream ss;
                    ss << "receive agent state from agent_" << msg->agent_id << 
                    ", agent state = " << msg->agent_state;
                    if(msg->agent_state == 0) {
                        agent_finishes_[msg->agent_id] = true;
                    }
                });        

        //sleep(1);  // wait to ensure all local agent will receive initial goal      

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this]() {
            //std::stringstream ss;
            //ss << "during CentralController loop, paused = " << paused_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            if(!paused_) {
                updateADG(); 
            }
        });

        // visualize all agent's unfinished path
        if(enable_opencv_window) {
            std::thread canvas_thread(PathVisualize, is_occupied);
            canvas_thread.detach();
        }

    }


    void responseCallback(rclcpp::Client<lamapf_and_gazebo_msgs::srv::LocalGoal>::SharedFuture future)
    {
        auto response = future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(),
                        "Service success");
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Service failed");
        }
    }

    void pubInitialGoals() {
        // pub all agent's first goal
        for(int i=0; i<instances_.first.size(); i++) {
            if(ADG_->paths_[i].size() < 2) {
                continue;
            }
            std::stringstream ss;

            size_t start_pose_id = ADG_->paths_[i][0];

            ss.str("");ss.clear();
            ss << "agent_" << i << ", start_pose_id = " << start_pose_id;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
            assert(start_pose != nullptr);
            Pointf<3> start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);

            size_t target_pose_id = ADG_->paths_[i][1];
            
            ss.str("");ss.clear();
            ss << "agent_" << i << ", target_pose_id = " << target_pose_id;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            PosePtr<int, 2> target_pose = ADG_->all_poses_[target_pose_id];
            assert(target_pose != nullptr);
            Pointf<3> target_ptf = pose_to_ptf_func_(*target_pose);//PoseIntToPtf(target_pose, grid_to_ptf_func_);

            // lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
            // msg.start_x   = start_ptf[0];
            // msg.start_y   = start_ptf[1];
            // msg.start_yaw = start_ptf[2];

            // msg.target_x   = target_ptf[0];
            // msg.target_y   = target_ptf[1];
            // msg.target_yaw = target_ptf[2];
            
            // msg.agent_id   = i;
            // msg.wait       = false;
            
            // goal_publishers_[i]->publish(msg);


            auto request = std::make_shared<lamapf_and_gazebo_msgs::srv::LocalGoal::Request>();


            request->start_x   = start_ptf[0];
            request->start_y   = start_ptf[1];
            request->start_yaw = start_ptf[2];

            request->goal_x    = target_ptf[0];
            request->goal_y    = target_ptf[1];
            request->goal_yaw  = target_ptf[2];

            request->wait = false;

            auto future = goal_pub_clients_[i]->async_send_request(
                request,
                std::bind(&CenteralControllerNew::responseCallback, this, std::placeholders::_1)
            );

            ss.str("");ss.clear();
            ss << "central controller pub agent_" << i << "'s initial goal " << start_ptf <<"->" << target_ptf;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

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
        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
        return layered_paths;
    }

    // update progress
    void updateADG() {

        if(pub_init_target_) {
            pubInitialGoals();
            pub_init_target_ = false;// only pub the first time
        }
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
                        RCLCPP_INFO(this->get_logger(), "agent_%i reach temporal target state", i);
                        // reach current target, move to next target
                        ADG_->setActionLeave(i, progress_of_agents_[i]);
                        // if next action is valid
                        if(!ADG_->isActionValid(i, progress_of_agents_[i] + 1)) {
                            RCLCPP_INFO(this->get_logger(), "agent_%i next action %i invalid", i, progress_of_agents_[i] + 1);
                            // tell the agent to wait utill action are valid
                            // lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                            // msg.agent_id   = i;
                            // msg.wait       = true;
                            
                            // goal_publishers_[i]->publish(msg); 

                            auto request = std::make_shared<lamapf_and_gazebo_msgs::srv::LocalGoal::Request>();

                            request->wait = true;

                            auto future = goal_pub_clients_[i]->async_send_request(
                                request,
                                std::bind(&CenteralControllerNew::responseCallback, this, std::placeholders::_1)
                            );

                            break;
                        }
                        RCLCPP_INFO(this->get_logger(), "agent_%i's progress update from % i/%i to %i/%i",
                                     i, progress_of_agents_[i], ADG_->paths_[i].size(),
                                     progress_of_agents_[i]+1, ADG_->paths_[i].size());
                        progress_of_agents_[i]++;
                        agent_finishes_[i] = false;
                        //RCLCPP_INFO(this->get_logger(), "flag 0.4");
                        if(progress_of_agents_[i] + 1 <= ADG_->paths_[i].size() - 1) {
                            //RCLCPP_INFO(this->get_logger(), "flag 0.5");
                            //RCLCPP_INFO(this->get_logger(), ss.str());
                            // update the agent's start and target state
                            size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
                            PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
                            start_ptf = pose_to_ptf_func_(*start_pose);//PoseIntToPtf(start_pose, grid_to_ptf_func_);

                            size_t target_pose_id = ADG_->paths_[i][progress_of_agents_[i] + 1];
                            PosePtr<int, 2> target_pose = ADG_->all_poses_[target_pose_id];
                            target_ptf = pose_to_ptf_func_(*target_pose);//PoseIntToPtf(target_pose, grid_to_ptf_func_);

                            // lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                            // msg.start_x   = start_ptf[0];
                            // msg.start_y   = start_ptf[1];
                            // msg.start_yaw = start_ptf[2];

                            // msg.target_x   = target_ptf[0];
                            // msg.target_y   = target_ptf[1];
                            // msg.target_yaw = target_ptf[2];

                            // msg.agent_id   = i;
                            // msg.wait       = false;
                            
                            // goal_publishers_[i]->publish(msg); 

                            auto request = std::make_shared<lamapf_and_gazebo_msgs::srv::LocalGoal::Request>();

                            request->start_x   = start_ptf[0];
                            request->start_y   = start_ptf[1];
                            request->start_yaw = start_ptf[2];

                            request->goal_x    = target_ptf[0];
                            request->goal_y    = target_ptf[1];
                            request->goal_yaw  = target_ptf[2];

                            request->wait = false;

                            auto future = goal_pub_clients_[i]->async_send_request(
                                request,
                                std::bind(&CenteralControllerNew::responseCallback, this, std::placeholders::_1)
                            );

                            std::stringstream ss;
                            ss << "central controller pub agent_" << i << "'s goal " << start_ptf <<"->" << target_ptf;
                            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

                        }
                    } else {
                        //RCLCPP_INFO(this->get_logger(), "flag 1.1");
                        break;
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "agent_%i 's task finish", i);
                    // when finish tell it to wait at target
                    
                    // lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                    // msg.agent_id   = i;
                    // msg.wait       = true;
                            
                    // goal_publishers_[i]->publish(msg); 

                    auto request = std::make_shared<lamapf_and_gazebo_msgs::srv::LocalGoal::Request>();

                    request->wait = true;

                    auto future = goal_pub_clients_[i]->async_send_request(
                        request,
                        std::bind(&CenteralControllerNew::responseCallback, this, std::placeholders::_1)
                    );

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
            RCLCPP_INFO(this->get_logger(), "all agents reach target");
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
        while(rclcpp::ok()) {
            canvas.resetCanvas();
            canvas.drawEmptyGrid();
            canvas.drawGridMap(dim_, is_occupied);
            // draw all agent's unfinished path
            for(int i=0; i<ADG_->paths_.size(); i++)
            {
                const auto& path = ADG_->paths_[i];
                if(path.empty()) { continue; }
                for(int t=progress_of_agents_[i]; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses_[path[t]]->pt_;
                    Pointi<2> pt2 = all_poses_[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1., zoom_ratio/10.), COLOR_TABLE[(i) % 30]);

                    // Pointf<3> ptf3 = pose_to_ptf_func_(*all_poses_[path[t]]);
                    // Pointf<3> ptf4 = pose_to_ptf_func_(*all_poses_[path[t+1]]);
                    // canvas.drawLine(ptf3[0], ptf3[1], ptf4[0], ptf4[1], std::max(1., zoom_ratio/10.), true, COLOR_TABLE[(i) % 30]);
                }
            }
            // draw every agent's pose
            for(int i=0; i<instances_.second.size(); i++)
            {
                //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                const auto &instance = instances_.second[i]; // zoom_ratio/10
                //std::cout << "Agent " << *instances.first[i] << "'s pose "  << allAgentPoses[i] << std::endl;
                //std::cout << "canvas.reso = " << canvas.resolution_ << std::endl;
                //std::cout << "canvas.zoom_ratio = " << canvas.zoom_ratio_ << std::endl;
                double x = all_agent_poses_[i][0], y = all_agent_poses_[i][1], orient = all_agent_poses_[i][2];

                
                instances_.first[i]->drawOnCanvas(Pointf<3>{x, y, orient}, canvas, COLOR_TABLE[i%30], false);
                
                //canvas.drawArrowInt(allAgentPoses[i].pt_[0], allAgentPoses[i].pt_[1], -orientToPi_2D(allAgentPoses[i].orient_), 1, std::max(1, zoom_ratio/10));
                //break;
            }
            char key = canvas.show();
            if(key == 32) {
                paused_ = !paused_;
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

    //std::vector<rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr> goal_publishers_;

    std::vector<rclcpp::Client<lamapf_and_gazebo_msgs::srv::LocalGoal>::SharedPtr> goal_pub_clients_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_subscriber_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::ErrorState>::SharedPtr error_state_subscriber_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::AgentState>::SharedPtr agent_state_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    static bool paused_;

    static bool pub_init_target_;

    static bool need_replan_;

    //GRID_TO_PTF_FUNC grid_to_ptf_func_;

    static POSE_TO_PTF_FUNC pose_to_ptf_func_;

};



#endif





