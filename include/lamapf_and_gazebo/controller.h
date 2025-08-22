#ifndef LAMAPF_CONTROLLER
#define LAMAPF_CONTROLLER

#include "common_interfaces.h"
#include "action_dependency_graph.h"

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

struct CenteralController {

    CenteralController(const std::vector<LAMAPF_Path>& paths,
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
    Pointfs<3> calculateCMD(Pointfs<3> poses, Pointfs<3> vels, float time_interval) {
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
struct SingleController {

    SingleController(const AgentPtr<2>& agent,
                     const LineFollowControllerPtr& line_ctl,
                     const RotateControllerPtr& rot_ctl,
                     const rclcpp::Node::SharedPtr& node_ptr):
                     agent_(agent),
                     line_ctl_(line_ctl),
                     rot_ctl_(rot_ctl),
                     node_ptr_(node_ptr)
    {
        
    }

    // calculate vel cmd for each agent
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) {
        Pointf<3> cmd_vel = {0,0,0};
        if(wait_) { return cmd_vel; }
        if(start_ptf_[2] != target_ptf_[2]) {
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
            //RCLCPP_INFO(node_ptr_->get_logger(), ss.str());
            
            cmd_vel = rot_ctl_->calculateCMD(pose, vel, time_interval);

        } else {
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

    bool wait_ = false;

    Pointf<3> start_ptf_, target_ptf_;

    AgentPtr<2> agent_;

    LineFollowControllerPtr line_ctl_;

    RotateControllerPtr rot_ctl_;

    rclcpp::Node::SharedPtr node_ptr_;
};


// maintain a action dependency graph, 
// tell single robots when to move, move to where and when stop
// 
struct CenteralControllerADG {

    CenteralControllerADG(const std::vector<LAMAPF_Path>& paths,
                          const std::vector<AgentPtr<2>>& agents,
                          const std::vector<PosePtr<int, 2> >& all_poses,
                          const rclcpp::Node::SharedPtr& node_ptr): 
                       ADG_(paths, agents, all_poses),
                       node_ptr_(node_ptr) {

        progress_of_agents_.resize(agents.size(), 0);

        assert(paths.size() == agents.size());

    }

    // calculate vel cmd for each agent
    Pointfs<3> calculateCMD(Pointfs<3> poses, Pointfs<3> vels, float time_interval) {
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
                    target_pose = ADG_.all_poses_[target_pose_id];
                    target_ptf = PoseIntToPtf(target_pose);
                    
                    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();
                    float angle_to_target = fmod(target_ptf[2]-cur_pose[2], 2*M_PI);
                    size_t start_pose_id = ADG_.paths_[i][progress_of_agents_[i]];;
                    // check whether reach current target, if reach, update progress to next pose
                    if(fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001) {
                        // reach current target, move to next target
                        ADG_.setActionLeave(i, progress_of_agents_[i]);
                        progress_of_agents_[i]++;
                        //RCLCPP_INFO(node_ptr_->get_logger(), ss.str());
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
                // if move to next pose, current pose and next pose must be different 
                assert(start_pose_id != target_pose_id); // yz:250412, what if the target repeated in path's end ?

                if(start_pose->pt_ == target_pose->pt_) {
                    // rotate
                } else {
                    // move forward
                }
                retv[i] = cmd_vel; 
            }
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

    std::vector<int> progress_of_agents_;

    rclcpp::Node::SharedPtr node_ptr_; // for print log

};


#endif





