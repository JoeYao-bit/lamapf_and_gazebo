#ifndef LOCAL_CONTROLLER
#define LOCAL_CONTROLLER

#include "common_interfaces.h"
#include "lamapf_and_gazebo_msgs/msg/update_pose.hpp"
#include "lamapf_and_gazebo_msgs/msg/update_goal.hpp"
#include "lamapf_and_gazebo_msgs/msg/error_state.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "sensor_msgs/msg/laser_scan.hpp"

// m/s, rad/s
struct MotionConfig {
    float max_v_x = .1, min_v_x = 0;
    float max_v_y = 0, min_v_y = 0;
    float max_v_w = .1*M_PI, min_v_w = -.1*M_PI;

    float max_a_x = 3.0, min_a_x = -3.;
    float max_a_y = 0, min_a_y = 0;
    float max_a_w = M_PI, min_a_w = -M_PI;

    bool is_nonholonomic = true;
};

bool reachPosition(const float& x, const float& y, const float& target_x, const float& target_y) {
    float dist_to_target_position = (Pointf<2>{x, y} - Pointf<2>{target_x, target_y}).Norm();
    return fabs(dist_to_target_position) < 0.03;                                            
}

float shortestAngularDistance(float a, float b) {
    float diff = std::fmod(a - b + M_PI, 2 * M_PI);
    if (diff < 0)
        diff += 2 * M_PI;
    diff -= M_PI;
    return diff;
}


bool reachOrientation(const float& orientation, const float& target_orientation) {
    float angle_to_target = shortestAngularDistance(orientation, target_orientation);
    // return fabs(dist_to_target) < 0.001 && fabs(angle_to_target) < 0.001;
    return fabs(angle_to_target) < 0.03;
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
    Pointf<3> pt1_, pt2_;

    virtual void reset() = 0;

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

    void reset() override{}
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

    void reset() override {}

};



class RotateController {
public:
   
    // rotate from ang1 to ang2
    RotateController(const MotionConfig& cfg) {
        cfg_ = cfg;
    }

    // poseposi_rot: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) = 0;

    virtual void reset() = 0;

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

    void reset() override {}

};

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

        if(reachPosition(pose[0], pose[1], pt2_[0], pt2_[1])) {
            finish_move_ = true;
            finish_rotate_ = true;
        }

        if(!finish_rotate_ && !finish_move_) {

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

                std::cout << "rotate 1 positive = " << rot_ctl_->posi_rot_ << ", retv w = " << retv << std::endl;

            } else {
                finish_rotate_ = true;
                //std::cout << "set finish_rotate_ = true" << std::endl;
            }
            //std::cout << "(2)finish_rotate_ = " << finish_rotate_ << std::endl;
        } else if(finish_rotate_ && !finish_move_) {
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
        } else if(finish_rotate_ && finish_move_) {
            // rotate target pose

            double ref_theta = std::fmod(pt2_[2], 2*M_PI);
            if(ref_theta < 0) { ref_theta += 2*M_PI; } 

            double cur_theta = std::fmod(pose[2], 2*M_PI);
            if(cur_theta < 0) { cur_theta += 2*M_PI; } 

            if(!reachOrientation(cur_theta, ref_theta)) {
                rot_ctl_->ang_ = ref_theta; 

                if(fabs(ref_theta - cur_theta) <= M_PI) {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = true; }
                    else { rot_ctl_->posi_rot_ = false; }
                } else {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = false; }
                    else { rot_ctl_->posi_rot_ = true; }                    
                }

                retv = rot_ctl_->calculateCMD(pose, vel, time_interval);

                std::cout << "rotate 2 positive = " << rot_ctl_->posi_rot_ << ", retv w = " << retv << std::endl;

            } else {
                std::cout << "finish all task" << std::endl;
            }
        }
        return retv;
    }

    void reset() override {
        finish_rotate_ = false;
        finish_move_ = false;
        rot_ctl_->reset();
    }

    bool finish_rotate_ = false, finish_move_ = false;

    RotateControllerPtr rot_ctl_;

};

typedef std::shared_ptr<TwoPhaseLineFollowController> TwoPhaseLineFollowControllerPtr;


class TwoPhaseLineFollowControllerReal : public TwoPhaseLineFollowController {
public:

    TwoPhaseLineFollowControllerReal(const MotionConfig& cfg) : TwoPhaseLineFollowController(cfg) {}

    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) override {
        Pointf<3> retv = {0, 0, 0};

        if(reachPosition(pose[0], pose[1], pt2_[0], pt2_[1])) {
            // rotate
            double ref_theta = std::fmod(pt2_[2], 2*M_PI);
            if(ref_theta < 0) { ref_theta += 2*M_PI; } 

            double cur_theta = std::fmod(pose[2], 2*M_PI);
            if(cur_theta < 0) { cur_theta += 2*M_PI; } 

            if(!reachOrientation(cur_theta, ref_theta)) {
                rot_ctl_->ang_ = ref_theta; 

                if(fabs(ref_theta - cur_theta) <= M_PI) {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = true; }
                    else { rot_ctl_->posi_rot_ = false; }
                } else {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = false; }
                    else { rot_ctl_->posi_rot_ = true; }                    
                }

                retv = rot_ctl_->calculateCMD(pose, vel, time_interval);

                if(fabs(retv[2]) < 0.1) {
                    if(retv[2]>0) { retv[2]=0.1; }
                    if(retv[2]<0) { retv[2]=-0.1; }
                }


                std::cout << "rotate 2 positive = " << rot_ctl_->posi_rot_ << ", retv v = " << retv << std::endl;

            } else {
                std::cout << "finish both rotate and move" << std::endl;
            }
        } else {
            Eigen::Vector2d ref_vec(pt2_[0] - pose[0], pt2_[1] - pose[1]);

            // if head to target, move forward
            double ref_theta = std::fmod(std::atan2(ref_vec.y(), ref_vec.x()), 2*M_PI);
            if(ref_theta < 0) { ref_theta += 2*M_PI; } 

            double cur_theta = std::fmod(pose[2], 2*M_PI);
            if(cur_theta < 0) { cur_theta += 2*M_PI; } 

            if(reachOrientation(cur_theta, ref_theta)) {
                double dist_to_end = ref_vec.norm();
                if(dist_to_end > cfg_.max_v_x*time_interval) {
                    retv[0] = cfg_.max_v_x;
                } else {
                    retv[0] = dist_to_end/time_interval;
                }
                std::cout << "move to target, retv = " << retv << std::endl;
            } else {
                rot_ctl_->ang_ = ref_theta; // update target angle

                if(fabs(ref_theta - cur_theta) <= M_PI) {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = true; }
                    else { rot_ctl_->posi_rot_ = false; }
                } else {
                    if(ref_theta > cur_theta) { rot_ctl_->posi_rot_ = false; }
                    else { rot_ctl_->posi_rot_ = true; }                    
                }

                retv = rot_ctl_->calculateCMD(pose, vel, time_interval);

                if(fabs(retv[2]) < 0.1) {
                    if(retv[2]>0) { retv[2]=0.1; }
                    if(retv[2]<0) { retv[2]=-0.1; }
                }

                std::cout << "rotate 1 positive = " << rot_ctl_->posi_rot_ << ", retv = " << retv << std::endl;
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
                    const TwoPhaseLineFollowControllerPtr& line_ctl,
                    int all_agent_size,
                    const float& time_interval = 0.1,
                    std::string pose_topic = "",
                    std::string goal_topic = "",
                    std::string laser_topic = "",
                    std::string cmdvel_topic = ""):
                     agent_(agent),
                     line_ctl_(line_ctl),
                     velcmd_(Pointf<3>{0,0,0}),
                     Node((std::string("agent_")+std::to_string(agent->id_)).c_str()) {


        // NOTICE: cache queue size must be large enough to cache all agent's pose or goal
        // otherwise some msg will be ignored

        error_state_publisher_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::ErrorState>("AgentErrorState", 10);

        pose_publisher_        = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>("PoseUpdate", 2*all_agent_size);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::stringstream ss3;
        if(goal_topic == "") {
            ss3 << "GoalUpdate" << agent_->id_; 
        } else {
            ss3 << goal_topic;
        }
        RCLCPP_INFO(this->get_logger(), "Agent %i's pose sub goal topic name %s", agent_->id_, ss3.str().c_str());

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

                        line_ctl_->reset();
                        
                        line_ctl_->pt1_ = start_ptf_;
                        line_ctl_->pt2_ = target_ptf_;

                        std::stringstream ss2;
                        ss2 << "in LocalController, receive goal, agent_" << agent_->id_ << " ";
                        ss2 << start_ptf_ << "->" << target_ptf_ << ", wait = " << wait_;
                        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
                    }
                });


        std::stringstream ss4;
        if(laser_topic == "") {
            ss4 << "LaseScan" << agent_->id_;
        } else {
            ss4 << laser_topic;
        }
        RCLCPP_INFO(this->get_logger(), "Agent %i's pose sub laser topic name %s", agent_->id_, ss4.str().c_str());

        laserscan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                ss4.str().c_str(), 10,
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    laserscan_msg_ = msg;
                });

        std::stringstream ss5;
        if(pose_topic == "") {
            ss5 << "AmclPose" << agent_->id_;
        } else {
            ss5 << pose_topic;
        }
        //std::cout << "Agent " << agent_->id_ << "'s pose sub ros topic name " << ss5.str() << std::endl;
        RCLCPP_INFO(this->get_logger(), "Agent %i's pose sub pose topic name %s", agent_->id_, ss5.str().c_str());

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                ss5.str().c_str(), 10,
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

                        
                    pose_msg_ = msg;

                    cur_pose_[0] = pose_msg_->pose.pose.position.x;
                    cur_pose_[1] = pose_msg_->pose.pose.position.y;

                    
                    lamapf_and_gazebo_msgs::msg::UpdatePose pose_msg;
                    pose_msg.x   = pose_msg_->pose.pose.position.x;
                    pose_msg.y   = pose_msg_->pose.pose.position.y;

                    // 获取姿态四元数
                    double qx = pose_msg_->pose.pose.orientation.x;
                    double qy = pose_msg_->pose.pose.orientation.y;
                    double qz = pose_msg_->pose.pose.orientation.z;
                    double qw = pose_msg_->pose.pose.orientation.w;

                    // 构建 tf2 四元数对象
                    tf2::Quaternion q(qx, qy, qz, qw);

                    // 用 Matrix3x3 提取 roll, pitch, yaw
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    
                    pose_msg.yaw = yaw;

                    cur_pose_[2] = yaw;

                    pose_msg.agent_id = this->agent_->id_;
                    this->pose_publisher_->publish(pose_msg);

                    std::stringstream ss_local;
                    ss_local << "agent " << agent_->id_ << " receive pose (" << msg->pose.pose.position.x << ", "
                    << msg->pose.pose.position.y << ", " << yaw << ")";
                    RCLCPP_INFO(this->get_logger(), ss_local.str().c_str());

                });        
        
        std::stringstream ss6;
        if(cmdvel_topic == "") {
            ss6 << "CmdVel" << agent_->id_;
        } else {
            ss6 << cmdvel_topic;
        }

        RCLCPP_INFO(this->get_logger(), "Agent %i's pose pub velcmd topic name %s", agent_->id_, ss6.str().c_str());

        cmd_vel_publisher_ =  this->create_publisher<geometry_msgs::msg::Twist>(ss6.str(), 10);

        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this,time_interval,agent]() {
            //std::stringstream ss;
            //ss << "during LocalController loop, agent_" << agent->id_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            //publishPoseMsgSim(time_interval);

            // if no pose_msg_ yet, try get tf as source of localization
            bool get_tf_localization = false;
            if(pose_msg_ == nullptr) {
                try {
                    RCLCPP_WARN(this->get_logger(), "no pose data, try get tf data");

                    // 获取最新 transform，阻塞最多50ms秒
                    auto transformStamped = tf_buffer_->lookupTransform(
                        "map", "base_footprint", tf2::TimePointZero,
                        std::chrono::milliseconds(50));

                    double x = transformStamped.transform.translation.x;
                    double y = transformStamped.transform.translation.y;

                    // geometry_msgs::Quaternion -> tf2::Quaternion
                    tf2::Quaternion q;
                    tf2::fromMsg(transformStamped.transform.rotation, q);

                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                    RCLCPP_INFO(this->get_logger(),
                                "When no amcl pose, from tf, get current pose: x=%.3f, y=%.3f, theta=%.3f rad",
                                x, y, yaw);
                    cur_pose_[0] = x;
                    cur_pose_[1] = y;
                    cur_pose_[2] = yaw;

                    get_tf_localization = true;
                }
                catch (tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for transform map -> base_footprint: %s", ex.what());
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                }
            }

            // after initialized calculate vel cmd and pub
            if(pose_msg_ != nullptr || get_tf_localization) {

                // // 位姿应尽量实时，位姿时刻与当前时间差距过大则不更新位姿
                // rclcpp::Time now = this->get_clock()->now();
                // // 消息时间
                // rclcpp::Time msg_time(pose_msg_->header.stamp);

                // // 计算纳秒差
                // int64_t diff_ns = (now - msg_time).nanoseconds();

                // // 转为毫秒（double 类型）
                // double diff_ms = diff_ns / 1e6;


                geometry_msgs::msg::Twist cmd_vel;

                // 设置线速度（前进 0.2 m/s）
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.linear.z = 0.0;

                // 设置角速度（逆时针旋转 0.5 rad/s）
                cmd_vel.angular.x = 0.0;
                cmd_vel.angular.y = 0.0;
                cmd_vel.angular.z = 0.0;


                // if(diff_ms > 500) {
                //     std::stringstream ss;
                //     ss << "during LocalController loop, agent_" << agent->id_<<", last pose is too old, " << diff_ms << "ms ago";
                //     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                //     // pub stop cmd
                //     cmd_vel_publisher_->publish(cmd_vel);
                // }

                velcmd_ = calculateCMD(cur_pose_, velcmd_, 1.0);
                // pub vel cmd
                cmd_vel.linear.x  = velcmd_[0];
                cmd_vel.angular.z = velcmd_[2];

                cmd_vel_publisher_->publish(cmd_vel);

                std::stringstream ss;
                ss << "agent_" << agent_->id_ << " pub vel cmd vx = " << velcmd_[0] << ", w = " << velcmd_[2];
                RCLCPP_INFO(this->get_logger(), ss.str().c_str()); 
            }
        });


        std::stringstream ss;
        ss << "init LocalController, agent_" << agent_->id_;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        
    }

    // whether the agent will collide with obstacle if rotate
    // check via laserscan
    // std_msgs/Header header
    // float32 angle_min
    // float32 angle_max
    // float32 angle_increment
    // float32 time_increment
    // float32 scan_time
    // float32 range_min
    // float32 range_max
    // float32[] ranges
    // float32[] intensities
    // need test
    bool hasCollideInRotate(float rotate_angle) const {
        if(agent_->type_ == "Block_2D") {
            std::shared_ptr<BlockAgent_2D> block_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent_);
            auto min_pt = block_ptr->min_pt_, max_pt = block_ptr->max_pt_;
            //bool collide = BlockAgentRotateCollisionCheck(min_pt, max_pt, rotate_angle, angle, dist, count== 1);
            // 遍历 ranges 数组
            for (size_t i = 0; i < laserscan_msg_->ranges.size(); i++) {
                float range = laserscan_msg_->ranges[i];
                float angle = laserscan_msg_->angle_min + i * laserscan_msg_->angle_increment;
                // 过滤无效点
                if (std::isnan(range) || std::isinf(range)) {
                    continue;
                }

                if(BlockAgentRotateCollisionCheck(min_pt, max_pt, rotate_angle, angle, range)) {
                    return true;
                }        
            }
        } else if(agent_->type_ == "Circle") {
            std::shared_ptr<CircleAgent<2>> circle_ptr = std::dynamic_pointer_cast<CircleAgent<2>>(agent_);
            // 遍历 ranges 数组
            for (size_t i = 0; i < laserscan_msg_->ranges.size(); i++) {
                float range = laserscan_msg_->ranges[i];
                float angle = laserscan_msg_->angle_min + i * laserscan_msg_->angle_increment;
                // 过滤无效点
                if (std::isnan(range) || std::isinf(range)) {
                    continue;
                }

                if(range <= circle_ptr->radius_) {
                    return true;
                }        
            }
        }
    }

    // whether the agent will collide with obstacle if move
    // check via laserscan
    // need test
    bool hasCollideInMove(float move_dist) const {
        if(agent_->type_ == "Block_2D") {
            std::shared_ptr<BlockAgent_2D> block_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent_);
            auto min_pt = block_ptr->min_pt_, max_pt = block_ptr->max_pt_;
            //bool collide = BlockAgentMoveCollisionCheck(min_pt, max_pt, move_dist, angle, dist);
            // 遍历 ranges 数组
            for (size_t i = 0; i < laserscan_msg_->ranges.size(); i++) {
                float range = laserscan_msg_->ranges[i];
                float angle = laserscan_msg_->angle_min + i * laserscan_msg_->angle_increment;
                // 过滤无效点
                if (std::isnan(range) || std::isinf(range)) {
                    continue;
                }

                if(BlockAgentMoveCollisionCheck(min_pt, max_pt, move_dist, angle, range)) {
                    return true;
                }        
            }
        } else if(agent_->type_ == "Circle") {
            std::shared_ptr<CircleAgent<2>> circle_ptr = std::dynamic_pointer_cast<CircleAgent<2>>(agent_);
            //bool collide = CircleAgentMoveCollisionCheck(r, mov_vec, angle, dist);
            // 遍历 ranges 数组
            auto mov_vec = Pointf<2>{move_dist, 0};
            for (size_t i = 0; i < laserscan_msg_->ranges.size(); i++) {
                float range = laserscan_msg_->ranges[i];
                float angle = laserscan_msg_->angle_min + i * laserscan_msg_->angle_increment;
                // 过滤无效点
                if (std::isnan(range) || std::isinf(range)) {
                    continue;
                }
                
                if(CircleAgentMoveCollisionCheck(circle_ptr->radius_, mov_vec, angle, range)) {
                    return true;
                }        
            }
        }
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
        cmd_vel = line_ctl_->calculateCMD(pose, vel, time_interval); ;
        return cmd_vel;
    }


    // TODO: 
    // ros service: update pose in central controller
    // ros service: call replan in central controller when encounter exception

    bool wait_ = true;
 
    Pointf<3> start_ptf_, target_ptf_;

    Pointf<3> velcmd_;

    AgentPtr<2> agent_;

    TwoPhaseLineFollowControllerPtr line_ctl_;

    rclcpp::Subscription<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr goal_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscriber_;

    sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg_ = nullptr;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_ = nullptr;

    Pointf<3> cur_pose_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::ErrorState>::SharedPtr error_state_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdatePose>::SharedPtr pose_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    int count_of_iter_ = 0;

};


// fake robot node
// receive init pose, cmd vel and pub real time pose
class FakeRobot : public rclcpp::Node {
public:

    FakeRobot(int agent_id,
              const Pointf<3>& init_pose,
              int all_agent_size,
              const float& time_interval = 0.1): 
              agent_id_(agent_id), cur_pose_(init_pose), rclcpp::Node((std::string("FakeRobot")+std::to_string(agent_id)).c_str()) {

        time_interval_ = time_interval;

        std::stringstream ss;
        ss << "FakeRobot" << agent_id_ << " initialized";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        
        std::stringstream ss4;
        ss4 << "AmclPose" << agent_id_;

        std::cout << "Agent " << agent_id_ << "'s pose pub ros topic name " << ss4.str() << std::endl;

        pose_ros_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(ss4.str().c_str(), 10);

        std::stringstream ss5;
        ss5 << "CmdVel" << agent_id_;
        cmdvel_subscriber_   = this->create_subscription<geometry_msgs::msg::Twist>(
        ss5.str().c_str(), 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
            cmdvel_msg_ = msg;

            auto pre_pose = cur_pose_;
            Pointf<3> velcmd;
            velcmd[0] = cmdvel_msg_->linear.x;
            velcmd[2] = cmdvel_msg_->angular.z;

            std::stringstream ss7;
            ss7 << "fake robot "<< agent_id_ <<" receive cmdvel(vx,w) = " << velcmd[0] << ", " << velcmd[2];
            RCLCPP_INFO(this->get_logger(), ss7.str().c_str());

            cur_pose_ = updateAgentPose(pre_pose, velcmd, this->time_interval_);

            
        });        

    
        // std::chrono::milliseconds(int(1000*time_interval))
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this]() {
            //std::stringstream ss;
            //ss << "during LocalController loop, agent_" << agent->id_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            //publishPoseMsgSim(time_interval);


            geometry_msgs::msg::PoseWithCovarianceStamped pose_ros_msg;

            pose_ros_msg.header.frame_id = "map";
            pose_ros_msg.header.stamp = this->now();

            pose_ros_msg.pose.pose.position.x = cur_pose_[0];
            pose_ros_msg.pose.pose.position.y = cur_pose_[1];
            pose_ros_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, cur_pose_[2]);

            pose_ros_msg.pose.pose.orientation.x = q.x();
            pose_ros_msg.pose.pose.orientation.y = q.y();
            pose_ros_msg.pose.pose.orientation.z = q.z();
            pose_ros_msg.pose.pose.orientation.w = q.w();

            this->pose_ros_publisher_->publish(pose_ros_msg);

            std::stringstream ss2;
            ss2 << "FakeRobot" << agent_id_ << " pub new position " << cur_pose_;
            RCLCPP_INFO(this->get_logger(), ss2.str().c_str());
        });

    }

    int agent_id_;

    Pointf<3> cur_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_ros_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_subscriber_;

    geometry_msgs::msg::Twist::SharedPtr cmdvel_msg_ = nullptr;

    rclcpp::TimerBase::SharedPtr timer_;

    float time_interval_;

};

#endif