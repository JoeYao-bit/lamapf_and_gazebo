#ifndef LAMAPF_CONTROLLER
#define LAMAPF_CONTROLLER

#include "common_interfaces.h"
#include "action_dependency_graph.h"
#include "lamapf_and_gazebo_msgs/msg/update_pose.hpp"
#include "lamapf_and_gazebo_msgs/msg/update_goal.hpp"

// m/s, rad/s
struct MotionConfig {
    double max_v_x = .5, min_v_x = 0;
    double max_v_y = 0, min_v_y = 0;
    double max_v_w = .5*M_PI, min_v_w = -.5*M_PI;

    double max_a_x = .5, min_a_x = -0.5;
    double max_a_y = 0, min_a_y = 0;
    double max_a_w = 1.*M_PI, min_a_w = -1.*M_PI;

    bool is_nonholonomic = true;
};


// there are two kinds of controller, follow a line and rotate
class LineFollowController {
public:
    // move from pt1 to pt2
    LineFollowController(const MotionConfig& cfg) {
        cfg_ = cfg;
    }

    // pose: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const = 0;

    MotionConfig cfg_;
    Pointf<2> pt1_, pt2_;

};

typedef std::shared_ptr<LineFollowController> LineFollowControllerPtr;


class ConstantLineFollowController : public LineFollowController {
public:

    ConstantLineFollowController(const MotionConfig& cfg) : LineFollowController(cfg) {}

    // set pt1_ and pt2_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const override {
        Pointf<3> retv = {0, 0, 0};
        double dist_to_end = sqrt(pow(pose[0]-pt2_[0], 2) + pow(pose[1]-pt2_[1], 2));
        if(dist_to_end > cfg_.max_v_x*time_interval) {
            retv[0] = cfg_.max_v_x;
        } else {
            retv[0] = dist_to_end/time_interval;
        }
        return retv;
    }
};


class RotateController {
 public:
   
    // rotate from ang1 to ang2
    RotateController(const MotionConfig& cfg) {
        cfg_ = cfg;
    }

    // poseposi_rot: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const = 0;

    MotionConfig cfg_;
    bool posi_rot_;
    float ang_;

};

typedef std::shared_ptr<RotateController> RotateControllerPtr;

class ConstantRotateController : public RotateController {
public:

    ConstantRotateController(const MotionConfig& cfg) : RotateController(cfg) {}

    // set posi_rot_ and ang_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const override {
        assert(ang_ >= 0 && ang_ <= 2*M_PI);

        Pointf<3> retv = {0, 0, 0};
        if(posi_rot_) {
            while(pose[2] > ang_) {
                pose[2] = pose[2] - 2*M_PI;
            }
            if(pose[2] + cfg_.max_v_w*time_interval < ang_) {
                retv[2] = cfg_.max_v_w;
            } else {
                retv[2] = (ang_ - pose[2])/time_interval;
            }
        } else {
            while(pose[2] < ang_) {
                pose[2] = pose[2] + 2*M_PI;
            }
            if(pose[2] + cfg_.min_v_w*time_interval > ang_) {
                retv[2] = cfg_.min_v_w;
            } else {
                retv[2] = (ang_ - pose[2])/time_interval;
            }
        }
        return retv;
    }

};



Pointf<3> updateAgentPose(Pointf<3> pose, Pointf<3> velcmd, float time_interval) {

    double new_x = pose[0] + time_interval*velcmd[0]*cos(pose[2]) - time_interval*velcmd[1]*sin(pose[2]), 

           new_y = pose[1] + time_interval*velcmd[0]*sin(pose[2]) + time_interval*velcmd[1]*cos(pose[2]),

           new_theta = fmod((pose[2] + time_interval*velcmd[2]) + 2*M_PI, 2*M_PI);

    return Pointf<3>{new_x, new_y, new_theta};
};

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
                    target_ptf = PoseIntToPtf(target_pose);
                    
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
                        //std::cout << "agent " << i  << " at pose " << poses[i] << " reach " << target_ptf << std::endl; 
                        //std::stringstream ss;
                        //ss << "flag 1.0" << ", agent " << i  << " at pose " << poses[i] << " reach " << target_ptf;
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
                    target_ptf = PoseIntToPtf(target_pose);
                    //std::cout << "agent " << i << " finish all pose, now at " << poses[i] << ", final pose = " << target_ptf << std::endl;
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
                start_ptf = PoseIntToPtf(start_pose);


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
                target_ptf = PoseIntToPtf(target_pose);
                //std::cout << "agent " << i << " finish2, at " << poses[i] << ", target = " << target_ptf << std::endl;
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


// for single robot, receive goal state and target state from CentralController
// run on single robot
// compute vel cmd, no decision, except detect route is obstructed
class LocalController : public rclcpp::Node {
public:

    LocalController(const AgentPtr<2>& agent,
                    const LineFollowControllerPtr& line_ctl,
                    const RotateControllerPtr& rot_ctl,
                    const float& time_interval = 0.1):
                     agent_(agent),
                     line_ctl_(line_ctl),
                     rot_ctl_(rot_ctl),
                     Node((std::string("agent_")+std::to_string(agent->id_)).c_str()) {
        // RCLCPP_INFO(this->get_logger(), "flag 1");
        pose_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>("PoseUpdate", 10);
        goal_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::UpdateGoal>(
                "GoalUpdate", 10,
                [this](lamapf_and_gazebo_msgs::msg::UpdateGoal::SharedPtr msg) {
                    if(msg->agent_id == agent_->id_) {    
                        std::stringstream ss;
                        ss << "in LocalController, receive goal, agent id = " << agent_->id_ << " ";
                        start_ptf_[0]  = msg->start_x;    
                        start_ptf_[1]  = msg->start_y;    
                        start_ptf_[2]  = msg->start_yaw;    
                        target_ptf_[0] = msg->target_x;    
                        target_ptf_[1] = msg->target_y;    
                        target_ptf_[2] = msg->target_yaw;  
                        wait_          = msg->wait;
                        ss << start_ptf_ << "->" << target_ptf_ << ", wait = " << wait_;
                        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                    }
                });

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this,time_interval,agent]() {
            std::stringstream ss;
            ss << "during LocalController loop, agent id = " << agent->id_;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            publishPoseMsgSim(time_interval);
        });
        std::stringstream ss;
        ss << "init LocalController, agent id = " << agent->id_;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    // calculate vel cmd for each agent
    Pointf<3> calculateCMD(const Pointf<3>& pose, const Pointf<3>& vel, const float& time_interval) {
        Pointf<3> cmd_vel = {0,0,0};
        // if is required to wait, then wait
        if(wait_) { return cmd_vel; }
        if(start_ptf_[2] != target_ptf_[2]) {
            // if out of current position (when rotate), stop and replan
            float dist_to_target = (Pointf<2>{target_ptf_[0], target_ptf_[1]} - Pointf<2>{pose[0], pose[1]}).Norm();
            if(dist_to_target > 0.1) {
                std::stringstream ss;
                ss << "current pose " << pose << " out of target position " << target_ptf_
                   << " in rotate, then stop and replan";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                return cmd_vel;
            }
            // rotate
            rot_ctl_->ang_ = target_ptf_[2]; // update target angle

            if(fabs(target_ptf_[2] - start_ptf_[2]) <= 0.5*M_PI + 0.001) {
                if(target_ptf_[2] > start_ptf_[2]) { rot_ctl_->posi_rot_ = true; }
                else { rot_ctl_->posi_rot_ = false; }
            } else {
                if(target_ptf_[2] > start_ptf_[2]) { rot_ctl_->posi_rot_ = false; }
                else { rot_ctl_->posi_rot_ = true; }                    
            }
            //std::stringstream ss;
            //ss << "current pose " << poses[i] << "start pose = " << start_ptf << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_;
            //RCLCPP_INFO(this->get_logger(), ss.str());
            
            cmd_vel = rot_ctl_->calculateCMD(pose, vel, time_interval);

        } else {
            // if out of current line (when move forward), stop and replan
            double dist_to_line = pointDistToLine(Pointf<2>{pose[0], pose[1]}, 
                                                  Pointf<2>{start_ptf_[0], start_ptf_[1]},
                                                  Pointf<2>{target_ptf_[0], target_ptf_[1]});
            if(dist_to_line > 0.1) { 
                std::stringstream ss;
                ss << "current pose " << pose << " out of target line " << start_ptf_ << "->" << target_ptf_
                   << " in move forward, then stop and replan";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                return cmd_vel;
            }
            // move forward
            line_ctl_->pt1_ = Pointf<2>({start_ptf_[0],  start_ptf_[1]});
            line_ctl_->pt2_ = Pointf<2>({target_ptf_[0], target_ptf_[1]}); // update target line
            //std::cout << "current pose " << poses[i] << ", target line = " << line_ctr->pt1_ << ", " << line_ctr->pt2_ << std::endl;
            cmd_vel = line_ctl_->calculateCMD(pose, vel, time_interval);
        }
        // TODO: predict whether there is obstacle in future path
        // if is, wait utill run out of time
        // if run out of time, call central controller to replan  
        return cmd_vel;
    }

    // publish simulated pose msg
    void publishPoseMsgSim(const float& time_interval) {
        velcmd_ = calculateCMD(cur_pose_, velcmd_, time_interval);
        // Pointf<3> pose, Pointf<3> velcmd, float time_interval
        cur_pose_ = updateAgentPose(cur_pose_, velcmd_, time_interval);
        lamapf_and_gazebo_msgs::msg::UpdatePose msg;
        msg.x   = cur_pose_[0];
        msg.y   = cur_pose_[1];
        msg.yaw = cur_pose_[2];
        msg.agent_id = agent_->id_;
        pose_publisher_->publish(msg);
        std::stringstream ss;
        ss << "agent " << agent_->id_ << " pub pose(x,y,yaw) = " << msg.x << ", " << msg.y << ", " << msg.yaw;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    // TODO: publish pose by localization
    void publishPoseMsg() const {
        //
    }

    // TODO: 
    // ros service: update pose in central controller
    // ros service: call replan in central controller when encounter exception

    bool wait_ = true;
 
    Pointf<3> start_ptf_, target_ptf_;

    AgentPtr<2> agent_;

    LineFollowControllerPtr line_ctl_;

    RotateControllerPtr rot_ctl_;

    Pointf<3> cur_pose_;

    Pointf<3> velcmd_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_publisher_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr goal_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

};


// maintain a action dependency graph, 
// tell single robots when to move, move to where and when stop
// 
class CenteralController : public rclcpp::Node  {
public:
    CenteralController(DimensionLength* dim, 
                       IS_OCCUPIED_FUNC<2> is_occupied,
                       const std::pair<AgentPtrs<2>, InstanceOrients<2> >& instances,
                       const float& time_interval = 0.1): 
                       dim_(dim),
                       isoc_(is_occupied),
                       instances_(instances),
                       rclcpp::Node("central_controller") {             
        
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
            Pointf<3> start_ptf = PoseIntToPtf(start_pose);
            all_agent_poses_[i] = start_ptf;
        }

        // RCLCPP_INFO(this->get_logger(), "flag 1");
        goal_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>("GoalUpdate", 10);
        pose_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>(
                "PoseUpdate", 10,
                [this](lamapf_and_gazebo_msgs::msg::UpdatePose::SharedPtr msg) {
                    std::stringstream ss;
                    ss << "during CentralController loop, receive pose of agent " << msg->agent_id;
                    all_agent_poses_[msg->agent_id][0] = msg->x;
                    all_agent_poses_[msg->agent_id][1] = msg->y;
                    all_agent_poses_[msg->agent_id][2] = msg->yaw;
                    ss << " = " << all_agent_poses_[msg->agent_id];
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                });

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this]() {
            std::stringstream ss;
            ss << "during CentralController loop";
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());

            updateADG(all_agent_poses_);

        });

    }

    // TODO: when single agent failed to complete its task, update all agent's path and search path again
    bool reupdate(const Pointfs<3>& poses) {
        // get current state as start of planning
        
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
        Pointf<3> start_ptf, target_ptf;
        bool all_finished = true;
        RCLCPP_INFO(this->get_logger(), "update ADG");
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
                    target_ptf = PoseIntToPtf(target_pose);
                    
                    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - 
                                                    Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();
                    float angle_to_target = fmod(target_ptf[2]-cur_pose[2], 2*M_PI);
                    // check whether reach current target, if reach, update progress to next pose
                    if(fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001) {
                        // if next action is valid
                        if(!ADG_->isActionValid(i, progress_of_agents_[i] + 1)) {
                            continue;
                        }
                        // reach current target, move to next target
                        ADG_->setActionLeave(i, progress_of_agents_[i]);
                        progress_of_agents_[i]++;
                        //RCLCPP_INFO(this->get_logger(), "flag 0.4");

                        //RCLCPP_INFO(this->get_logger(), ss.str());
                        // TODO: update the agent's start and target state
                        size_t start_pose_id = ADG_->paths_[i][progress_of_agents_[i]];
                        PosePtr<int, 2> start_pose = ADG_->all_poses_[start_pose_id];
                        start_ptf = PoseIntToPtf(start_pose);

                        size_t target_pose_id = ADG_->paths_[i][progress_of_agents_[i] + 1];
                        PosePtr<int, 2> target_pose = ADG_->all_poses_[target_pose_id];
                        target_ptf = PoseIntToPtf(target_pose);

                        lamapf_and_gazebo_msgs::msg::UpdateGoal msg;
                        msg.start_x   = start_ptf[0];
                        msg.start_y   = start_ptf[1];
                        msg.start_yaw = start_ptf[2];

                        msg.target_x   = target_ptf[0];
                        msg.target_y   = target_ptf[1];
                        msg.target_yaw = target_ptf[2];

                        msg.wait       = true;
                        
                        std::stringstream ss;
                        ss << "central controller pub agent " << i << "'s goal " << start_ptf <<"->" << target_ptf;
                        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                        goal_publisher_->publish(msg);
                    } else {
                        //RCLCPP_INFO(this->get_logger(), "flag 1.1");
                        break;
                    }
                } else {
                    //RCLCPP_INFO(this->get_logger(), "flag 2");
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

    DimensionLength* dim_;

    IS_OCCUPIED_FUNC<2> isoc_;

    std::pair<AgentPtrs<2>, InstanceOrients<2> > instances_;

    std::vector<std::shared_ptr<Pose<int, 2>> > all_poses_;

    std::shared_ptr<ActionDependencyGraph<2> > ADG_;

    Pointfs<3> all_agent_poses_;

    std::vector<int> progress_of_agents_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr goal_publisher_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

};


#endif





