#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

#include "common_interfaces.h"
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

bool reachTarget(const Pointf<3>& cur_pose, const Pointf<3>& target_ptf) {
    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - 
                                                    Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();
    float angle_to_target = fmod(target_ptf[2]-cur_pose[2], 2*M_PI);
    return fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001;
}

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


Pointf<3> updateAgentPose(const Pointf<3>& pose, const Pointf<3>& velcmd, float time_interval) {

    double new_x = pose[0] + time_interval*velcmd[0]*cos(pose[2]) - time_interval*velcmd[1]*sin(pose[2]), 

           new_y = pose[1] + time_interval*velcmd[0]*sin(pose[2]) + time_interval*velcmd[1]*cos(pose[2]),

           new_theta = fmod((pose[2] + time_interval*velcmd[2]) + 2*M_PI, 2*M_PI);

    return Pointf<3>{new_x, new_y, new_theta};
};


// for single robot, receive goal state and target state from CentralController
// run on single robot
// compute vel cmd, no decision, except detect route is obstructed
class LocalController : public rclcpp::Node {
public:

    LocalController(const AgentPtr<2>& agent,
                    const LineFollowControllerPtr& line_ctl,
                    const RotateControllerPtr& rot_ctl,
                    const Pointf<3>& init_pose,
                    int all_agent_size,
                    const float& time_interval = 0.1):
                     agent_(agent),
                     line_ctl_(line_ctl),
                     rot_ctl_(rot_ctl),
                     init_pose_(init_pose),
                     cur_pose_(init_pose),
                     Node((std::string("agent_")+std::to_string(agent->id_)).c_str()) {


        std::stringstream ss1;
        ss1 << "agent " << agent->id_ << " init pose " << cur_pose_;                
        RCLCPP_INFO(this->get_logger(), ss1.str().c_str());

        // NOTICE: cache queue size must be large enough to cache all agent's pose or goal
        // otherwise some msg will be ignored
        pose_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>("PoseUpdate", 2*all_agent_size);
        std::stringstream ss3;
        ss3 << "GoalUpdate" << agent->id_;
        goal_subscriber_ = this->create_subscription<lamapf_and_gazebo_msgs::msg::UpdateGoal>(
                ss3.str().c_str(), 2*all_agent_size,
                [this](lamapf_and_gazebo_msgs::msg::UpdateGoal::SharedPtr msg) {
                    if(msg->agent_id == agent_->id_) {    
                        start_ptf_[0]  = msg->start_x;    
                        start_ptf_[1]  = msg->start_y;    
                        start_ptf_[2]  = msg->start_yaw;    
                        target_ptf_[0] = msg->target_x;    
                        target_ptf_[1] = msg->target_y;    
                        target_ptf_[2] = msg->target_yaw;  
                        wait_          = msg->wait;

                        std::stringstream ss2;
                        ss2 << "in LocalController, receive goal, agent id = " << agent_->id_ << " ";
                        ss2 << start_ptf_ << "->" << target_ptf_ << ", wait = " << wait_;
                        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
                    }
                });

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this,time_interval,agent]() {
            //std::stringstream ss;
            //ss << "during LocalController loop, agent id = " << agent->id_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
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
        if(reachTarget(pose, target_ptf_)) {
            wait_ = true;
            return cmd_vel;
        }
        if(start_ptf_[2] != target_ptf_[2]) {
            // if out of current position (when rotate), stop and replan
            float dist_to_target = (Pointf<2>{target_ptf_[0], target_ptf_[1]} - Pointf<2>{pose[0], pose[1]}).Norm();
            if(dist_to_target > 0.1) {
                std::stringstream ss;
                ss << "current pose " << pose << " out of target position " << target_ptf_
                   << " in rotate, then stop and replan";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                sleep(1);
                assert(0);
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
            // std::stringstream ss;
            // ss << "current pose " << poses[i] << "start pose = " << start_ptf << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_;
            // RCLCPP_INFO(this->get_logger(), ss.str());
            
            // std::cout << "**agent vel cmd, " << agent_->id_ << ", current pose " << pose << "start pose = " << start_ptf_ << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_ << std::endl;

            cmd_vel = rot_ctl_->calculateCMD(pose, vel, time_interval);

        } else {
            // if out of current line (when move forward), stop and replan
            double dist_to_line = pointDistToLine(Pointf<2>{pose[0], pose[1]}, 
                                                  Pointf<2>{start_ptf_[0], start_ptf_[1]},
                                                  Pointf<2>{target_ptf_[0], target_ptf_[1]});
            if(dist_to_line > 0.1) { 
                std::stringstream ss;
                ss << "current pose " << pose << " out of target line " << start_ptf_ << "->" << target_ptf_
                   << " in move forward, then stop and replan, dist_to_line = " << dist_to_line;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                sleep(1);
                assert(0);
                return cmd_vel;
            }
            double dist_to_direct = fabs(target_ptf_[2] - pose[2]);                                      
            if(dist_to_direct > 0.1) { 
                std::stringstream ss;
                ss << "current pose " << pose << "'s direction out of target line " << start_ptf_ << "->" << target_ptf_
                   << " in move forward, then stop and replan, dist_to_direct = " << dist_to_direct;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                sleep(1);
                assert(0);
                return cmd_vel;
            }
            // move forward
            line_ctl_->pt1_ = Pointf<2>({start_ptf_[0],  start_ptf_[1]});
            line_ctl_->pt2_ = Pointf<2>({target_ptf_[0], target_ptf_[1]}); // update target line

            std::cout << "**agent vel cmd, " << agent_->id_ << ", current pose " << pose << ", target line = " << line_ctl_->pt1_ << ", yaw= " << start_ptf_[2] << ", " << line_ctl_->pt2_ << ", yaw = " << target_ptf_[2] << std::endl;

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
        auto pre_pose = cur_pose_;
        cur_pose_ = updateAgentPose(cur_pose_, velcmd_, time_interval);
        if(agent_->id_ == 9) {
            std::cout << "agent id = " << agent_->id_ << ", velcmd_ = " << velcmd_ << ", pre pose = "
                  << pre_pose << ", cur pose = " << cur_pose_ << std::endl;
        }
        lamapf_and_gazebo_msgs::msg::UpdatePose msg;
        msg.x   = cur_pose_[0];
        msg.y   = cur_pose_[1];
        msg.yaw = cur_pose_[2];
        msg.agent_id = agent_->id_;
        pose_publisher_->publish(msg);

        // std::stringstream ss;
        // ss << "agent " << agent_->id_ << " pub pose(x,y,yaw) = " << msg.x << ", " << msg.y << ", " << msg.yaw;
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
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

    Pointf<3> init_pose_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_publisher_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr goal_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

};





#endif