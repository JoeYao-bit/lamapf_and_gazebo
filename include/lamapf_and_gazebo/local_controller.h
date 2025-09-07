#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

#include "common_interfaces.h"
#include "lamapf_and_gazebo_msgs/msg/update_pose.hpp"
#include "lamapf_and_gazebo_msgs/msg/update_goal.hpp"
#include "lamapf_and_gazebo_msgs/msg/error_state.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>

// m/s, rad/s
struct MotionConfig {
    float max_v_x = .5, min_v_x = 0;
    float max_v_y = 0, min_v_y = 0;
    float max_v_w = .5*M_PI, min_v_w = -.5*M_PI;

    float max_a_x = 3.0, min_a_x = -3.;
    float max_a_y = 0, min_a_y = 0;
    float max_a_w = 2*M_PI, min_a_w = -2*M_PI;

    bool is_nonholonomic = true;
};

bool reachPosition(const float& x, const float& y, const float& target_x, const float& target_y) {
    float dist_to_target_position = (Pointf<2>{x, y} - Pointf<2>{target_x, target_y}).Norm();
    return fabs(dist_to_target_position) < 0.03;                                            
}

bool reachOrientation(const float& orientation, const float& target_orientation) {
    float angle_to_target = fmod(orientation-target_orientation, 2*M_PI);
    // return fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001;
    if(angle_to_target <= M_PI) {
        return fabs(angle_to_target) < 0.03;
    } else {
        return fabs(2*M_PI - angle_to_target) < 0.03;   
    }
}

bool reachTarget(const Pointf<3>& cur_pose, const Pointf<3>& target_ptf) {
    return reachPosition(cur_pose[0], cur_pose[1], target_ptf[0], target_ptf[1]) && reachOrientation(cur_pose[2], target_ptf[2]);
}

// there are two kinds of controller, follow a line and rotate
class LineFollowController {
public:
    // move from pt1 to pt2
    LineFollowController(const MotionConfig& cfg) {
        cfg_ = cfg;
    }

    // pose: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) = 0;

    MotionConfig cfg_;
    Pointf<2> pt1_, pt2_;

    // used only in two phase line follower
    bool finish_rotate_ = false;

};

typedef std::shared_ptr<LineFollowController> LineFollowControllerPtr;


class ConstantLineFollowController : public LineFollowController {
public:

    ConstantLineFollowController(const MotionConfig& cfg) : LineFollowController(cfg) {}

    // set pt1_ and pt2_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) override {
        Pointf<3> retv = {0, 0, 0};
        double dist_to_end = sqrt(pow(pose[0]-pt2_[0], 2) + pow(pose[1]-pt2_[1], 2));
        if(dist_to_end > cfg_.max_v_x*time_interval) {
            retv[0] = cfg_.max_v_x;
        } else {
            retv[0] = dist_to_end/time_interval;
        }

        // retv[0] = std::clamp(retv[0],
        //         vel[0] + cfg_.min_a_x * time_interval,
        //         vel[0] + cfg_.max_a_x * time_interval);

        return retv;
    }
};

// have no linear velocity
class MPCLineFollowController : public LineFollowController {
public:
    MPCLineFollowController(const MotionConfig& cfg) : LineFollowController(cfg) {}

    // set pt1_ and pt2_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) override {
        Pointf<3> retv = {0, 0, 0};

        // -----------------------------
        // 1. 参考方向
        // -----------------------------
        Eigen::Vector2f ref_vec(pt2_[0] - pt1_[0], pt2_[1] - pt1_[1]);
        //Eigen::Vector2d ref_vec(pt2_[0] - pose[0], pt2_[1] - pose[1]);

        float ref_theta = std::atan2(ref_vec.y(), ref_vec.x());

        // -----------------------------
        // 2. 误差计算 (相对于终点)
        // -----------------------------
        Eigen::Vector2f pos_error(pose[0] - pt2_[0], pose[1] - pt2_[1]);

        //std::cout << "pos_error = " << pos_error[0] << ", " << pos_error[1] << std::endl;

        float e_long  =  pos_error.x() * std::cos(ref_theta) + pos_error.y() * std::sin(ref_theta);
        float e_lat   = -pos_error.x() * std::sin(ref_theta) + pos_error.y() * std::cos(ref_theta);
        float e_theta = pose[2] - ref_theta;

        //std::cout << "e_long = " << e_long << ", e_lat = " << e_lat << ", e_theta = " << e_theta << std::endl;

        // -----------------------------
        // 3. 成本函数权重
        // -----------------------------
        float alpha = 0.01;  // 纵向误差权重
        float beta  = 0.1; // 横向误差权重
        float gamma = 0.01; // 航向误差权重

        // -----------------------------
        // 4. 简单 QP 近似解 (代替完整优化器)
        // -----------------------------
        Eigen::Matrix2f H;
        H << 2 * alpha, 0,
                0, 2 * gamma;

        Eigen::Vector2f g;
        g << 2 * alpha * e_long + 2 * beta * e_lat,   // 线速度受纵向和横向误差影响
             2 * gamma * e_theta;                     // 角速度受航向误差影响


        Eigen::Vector2f u_star = -H.ldlt().solve(g); // [v, w]

        float v_cmd = u_star[0];
        float w_cmd = u_star[1];

        //std::cout << "raw v_cmd = " << v_cmd << ", w_cmd = " << w_cmd << std::endl;

        // -----------------------------
        // 5. 速度限制 (MotionConfig)
        // -----------------------------
        // std::clamp 裁减到速度上下限之间
        v_cmd = std::clamp(v_cmd, cfg_.min_v_x, cfg_.max_v_x);
        w_cmd = std::clamp(w_cmd, cfg_.min_v_w, cfg_.max_v_w);

        //std::cout << "clamp v_cmd = " << v_cmd << ", w_cmd = " << w_cmd << std::endl;

        // -----------------------------
        // 6. 输出 [vx, vy, w]
        // 差速小车: 只有 vx 和 w
        // -----------------------------
        retv[0] = v_cmd;
        retv[1] = 0.0;
        retv[2] = w_cmd;


        // ------------------------
        // 加速度限制
        // ------------------------
        // float v_delta_max = cfg_.max_a_x * time_interval;
        // float w_delta_max = cfg_.max_a_w * time_interval;

        // std::clamp 是 C++17 引入的一个小工具函数，用来把数值限制在某个区间内。
        // constexpr const T& clamp( const T& v, const T& lo, const T& hi );
        // v → 需要被限制的值，lo → 下限，hi → 上限（要求 lo <= hi）
        float v_next = std::clamp(v_cmd,
                                    vel[0] + cfg_.min_a_x * time_interval,
                                    vel[0] + cfg_.max_a_x * time_interval);

        float w_next = std::clamp(w_cmd,
                                    vel[2] + cfg_.min_a_w * time_interval,
                                    vel[2] + cfg_.max_a_w * time_interval);

        // 输出结果
        retv[0] = v_next;   // 前进线速度
        retv[1] = 0.0;      // 差动小车不考虑 y 方向
        retv[2] = w_next;   // 角速度

        //std::cout << "limit acc v_cmd = " << v_next << ", w_cmd = " << w_next << std::endl;
        
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
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) = 0;

    MotionConfig cfg_;
    bool posi_rot_;
    float ang_;

};

typedef std::shared_ptr<RotateController> RotateControllerPtr;

class ConstantRotateController : public RotateController {
public:

    ConstantRotateController(const MotionConfig& cfg) : RotateController(cfg) {}

    // set posi_rot_ and ang_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) override {
        //assert(ang_ >= 0 && ang_ <= 2*M_PI);
        float ang = std::fmod(ang_, 2*M_PI);
        Pointf<3> retv = {0, 0, 0};
        if(posi_rot_) {
            while(pose[2] > ang) {
                pose[2] = pose[2] - 2*M_PI;
            }
            //std::cout << "posi_rot_ " << posi_rot_ << ", (ang - pose[2]) = " << ang - pose[2] << std::endl;
            retv[2] = std::min(cfg_.max_v_w, (ang - pose[2])/time_interval);
        } else {
            while(pose[2] < ang) {
                pose[2] = pose[2] + 2*M_PI;
            }
            //std::cout << "posi_rot_ " << posi_rot_ << ", (ang - pose[2]) = " << ang - pose[2] << std::endl;
            retv[2] = std::max(cfg_.min_v_w, (ang - pose[2])/time_interval);
        }

        // retv[2] = std::clamp(retv[2],
        //         vel[2] + cfg_.min_a_w * time_interval,
        //         vel[2] + cfg_.max_a_w * time_interval);

        return retv;
    }

};

// have no linear velocity
class TwoPhaseLineFollowController : public LineFollowController {
public:
    TwoPhaseLineFollowController(const MotionConfig& cfg) 
    : LineFollowController(cfg), rot_ctl_(std::make_shared<ConstantRotateController>(cfg)) {}

    // set pt1_ and pt2_ before call calculateCMD
    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) override {
        Pointf<3> retv = {0, 0, 0};

        // rotate head to target
        Eigen::Vector2d ref_vec(pt2_[0] - pose[0], pt2_[1] - pose[1]);
        //std::cout << "(1)finish_rotate_ = " << finish_rotate_ << std::endl;
        if(!finish_rotate_) {

            double ref_theta = std::fmod(std::atan2(ref_vec.y(), ref_vec.x()), 2*M_PI);
            if(ref_theta < 0) { ref_theta += 2*M_PI; } 

            double cur_theta = std::fmod(pose[2], 2*M_PI);
            if(cur_theta < 0) { cur_theta += 2*M_PI; } 

            if(!reachOrientation(cur_theta, ref_theta)) {
                                // otherwise rotate to target direction
                rot_ctl_->ang_ = ref_theta; // update target angle

                if(fabs(ref_theta - cur_theta) <= M_PI) {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = true; }
                    else { rot_ctl_->posi_rot_ = false; }
                } else {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = false; }
                    else { rot_ctl_->posi_rot_ = true; }                    
                }

                retv = rot_ctl_->calculateCMD(pose, vel, time_interval);

                std::cout << "rotate positive = " << rot_ctl_->posi_rot_ << ", retv w = " << retv << std::endl;

            } else {
                finish_rotate_ = true;
                //std::cout << "set finish_rotate_ = true" << std::endl;
            }
            //std::cout << "(2)finish_rotate_ = " << finish_rotate_ << std::endl;
        } else if(!reachPosition(pose[0], pose[1], pt2_[0], pt2_[1])) {
            // move to target
            double dist_to_end = ref_vec.norm();
            if(dist_to_end > cfg_.max_v_x*time_interval) {
                retv[0] = cfg_.max_v_x;
            } else {
                retv[0] = dist_to_end/time_interval;
            }

            //std::cout << "dist_to_end = " << dist_to_end << ", retv v = " << retv << std::endl;

            // retv[0] = std::clamp(retv[0],
            //                             vel[0] + cfg_.min_a_x * time_interval,
            //                             vel[0] + cfg_.max_a_x * time_interval);
        } else {
            //std::cout << "finish all task" << std::endl;
        }
        return retv;
    }

    RotateControllerPtr rot_ctl_;

};

Pointf<3> updateAgentPose(const Pointf<3>& pose, const Pointf<3>& velcmd, float time_interval) {

    double new_x = pose[0] + time_interval*velcmd[0]*cos(pose[2]) - time_interval*velcmd[1]*sin(pose[2]), 

           new_y = pose[1] + time_interval*velcmd[0]*sin(pose[2]) + time_interval*velcmd[1]*cos(pose[2]),

           new_theta = fmod((pose[2] + time_interval*velcmd[2]) + 2*M_PI, 2*M_PI);

    return Pointf<3>{new_x, new_y, new_theta};

    // double x = pose[0];
    // double y = pose[1];
    // double theta = pose[2];

    // double v = velcmd[0];   // 前向速度
    // double vy = velcmd[1];  // 横向速度（若用单轨，可设为 0）
    // double w = velcmd[2];   // 角速度

    // double new_x, new_y, new_theta;

    // if (std::fabs(w) < 1e-6) {
    //     // ω≈0，退化为直线运动
    //     new_x = x + (v * std::cos(theta) - vy * std::sin(theta)) * time_interval;
    //     new_y = y + (v * std::sin(theta) + vy * std::cos(theta)) * time_interval;
    // } else {
    //     // 精确积分（指数映射）
    //     double sin_dt = std::sin(w * time_interval);
    //     double cos_dt = std::cos(w * time_interval);

    //     new_x = x + ( v * sin_dt + vy * (cos_dt - 1.0)) / w * std::cos(theta)
    //               + ( v * (1.0 - cos_dt) + vy * sin_dt ) / w * (-std::sin(theta));

    //     new_y = y + ( v * sin_dt + vy * (cos_dt - 1.0)) / w * std::sin(theta)
    //               + ( v * (1.0 - cos_dt) + vy * sin_dt ) / w * ( std::cos(theta));
    // }

    // new_theta = std::fmod(theta + w * time_interval + 2*M_PI, 2*M_PI);

    // return Pointf<3>{new_x, new_y, new_theta};

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
        ss1 << "agent_" << agent_->id_ << " init pose " << cur_pose_;                
        RCLCPP_INFO(this->get_logger(), ss1.str().c_str());

        // NOTICE: cache queue size must be large enough to cache all agent's pose or goal
        // otherwise some msg will be ignored
        pose_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>("PoseUpdate", 2*all_agent_size);

        error_state_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::ErrorState>("AgentErrorState", 10);

        std::stringstream ss3;
        ss3 << "GoalUpdate" << agent_->id_;
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

                        line_ctl_->finish_rotate_ = false;
                        std::stringstream ss2;
                        ss2 << "in LocalController, receive goal, agent_" << agent_->id_ << " ";
                        ss2 << start_ptf_ << "->" << target_ptf_ << ", wait = " << wait_;
                        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
                    }
                });

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this,time_interval,agent]() {
            //std::stringstream ss;
            //ss << "during LocalController loop, agent_" << agent->id_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            publishPoseMsgSim(time_interval);
        });
        std::stringstream ss;
        ss << "init LocalController, agent_" << agent_->id_;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        
    }

    // calculate vel cmd for each agent
    Pointf<3> calculateCMD(const Pointf<3>& pose, const Pointf<3>& vel, const float& time_interval) {
        count_of_iter_ ++;
        Pointf<3> cmd_vel = {0,0,0};
        // if is required to wait, then wait
        if(wait_) { return cmd_vel; }
        if(reachTarget(pose, target_ptf_)) {
            wait_ = true;
            return cmd_vel;
        }
        // if(agent_->id_ == 0 
        //     && count_of_iter_ == 30
        // ) {
        //     std::stringstream ss;
        //     ss << "pub fake error state, count_of_iter_ = " << count_of_iter_;
        //     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        //     lamapf_and_gazebo_msgs::msg::ErrorState msg;
        //     msg.agent_id = agent_->id_;
        //     msg.error_state = 0;
        //     error_state_publisher_->publish(msg);
        // }                                  
        // if current orientation is not the pose's direction 

        
        if(start_ptf_[2] != target_ptf_[2]) {
            // if out of current position (when rotate), stop and replan
            float dist_to_target = (Pointf<2>{target_ptf_[0], target_ptf_[1]} - Pointf<2>{pose[0], pose[1]}).Norm();
            if(dist_to_target > 0.1) {
                std::stringstream ss;
                ss << "agent_" << agent_->id_ << ", current pose " << pose << " out of target position " << target_ptf_
                   << " in rotate, then stop and replan";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                wait_ = true;
                lamapf_and_gazebo_msgs::msg::ErrorState msg;
                msg.agent_id = agent_->id_;
                msg.error_state = 0;
                error_state_publisher_->publish(msg);
                //sleep(1);
                //assert(0);
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
            std::stringstream ss;
            ss << "agent_" << agent_->id_ << ", current pose " << pose << "start pose = " << start_ptf_ << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            
            std::cout << "**vel cmd rot, agent_" << agent_->id_ << ", current pose " << pose << ", start pose = " << start_ptf_ << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_ << std::endl;

            cmd_vel = rot_ctl_->calculateCMD(pose, vel, time_interval);

        } else {
            if(!reachOrientation(pose[2], start_ptf_[2])) {
                // rotate
                rot_ctl_->ang_ = start_ptf_[2]; // update target angle

                if(fabs(start_ptf_[2] - pose[2]) <= 0.5*M_PI + 0.001) {
                    if(start_ptf_[2] > pose[2]) { rot_ctl_->posi_rot_ = true; }
                    else { rot_ctl_->posi_rot_ = false; }
                } else {
                    if(start_ptf_[2] > pose[2]) { rot_ctl_->posi_rot_ = false; }
                    else { rot_ctl_->posi_rot_ = true; }                    
                }
                std::stringstream ss;
                ss << "agent_" << agent_->id_ << ", current pose " << pose << "start pose = " << start_ptf_ << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                
                std::cout << "**vel cmd rot, agent_" << agent_->id_ << ", current pose " << pose << ", start pose = " << start_ptf_ << ", target dir/ang = " << rot_ctl_->posi_rot_ << ", " << rot_ctl_->ang_ << std::endl;

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
                    lamapf_and_gazebo_msgs::msg::ErrorState msg;
                    msg.agent_id = agent_->id_;
                    msg.error_state = 0;
                    error_state_publisher_->publish(msg);
                    // sleep(1);
                    // assert(0);
                    return cmd_vel;
                }
                double dist_to_direct = std::fmod(target_ptf_[2] - pose[2], 2*M_PI);   
                if(dist_to_direct > 0.1) { 
                    std::stringstream ss;
                    ss << "current pose " << pose << "'s direction out of target line " << start_ptf_ << "->" << target_ptf_
                    << " in move forward, then stop and replan, dist_to_direct = " << dist_to_direct;
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                    wait_ = true;
                    lamapf_and_gazebo_msgs::msg::ErrorState msg;
                    msg.agent_id = agent_->id_;
                    msg.error_state = 0;
                    error_state_publisher_->publish(msg); 
                    // sleep(1);
                    // assert(0);
                    return cmd_vel;
                }
                // move forward
                line_ctl_->pt1_ = Pointf<2>({start_ptf_[0],  start_ptf_[1]});
                line_ctl_->pt2_ = Pointf<2>({target_ptf_[0], target_ptf_[1]}); // update target line

                std::cout << "**vel cmd mov, agent_" << agent_->id_ << ", current pose " << pose << ", target line = " << line_ctl_->pt1_ << ", yaw= " << start_ptf_[2] << ", " << line_ctl_->pt2_ << ", yaw = " << target_ptf_[2] << std::endl;

                cmd_vel = line_ctl_->calculateCMD(pose, vel, time_interval); 
            }
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
        // if(agent_->id_ == 9) {
        //     std::cout << "agent_id = " << agent_->id_ << ", velcmd_ = " << velcmd_ << ", pre pose = "
        //           << pre_pose << ", cur pose = " << cur_pose_ << std::endl;
        // }
        lamapf_and_gazebo_msgs::msg::UpdatePose msg;
        msg.x   = cur_pose_[0];
        msg.y   = cur_pose_[1];
        msg.yaw = cur_pose_[2];
        msg.agent_id = agent_->id_;
        pose_publisher_->publish(msg);

        if(!wait_) {
            std::stringstream ss;
            ss << "agent_" << agent_->id_ << ", cmdvel = " << velcmd_ << ", pub pose(x,y,yaw) = " << msg.x << ", " << msg.y << ", " << msg.yaw;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str()); 
        }
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

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::ErrorState>::SharedPtr error_state_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    int count_of_iter_ = 0;

};





#endif