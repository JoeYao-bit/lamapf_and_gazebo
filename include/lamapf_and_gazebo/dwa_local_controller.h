#ifndef DWA_LOCAL_CONTROLLER
#define DWA_LOCAL_CONTROLLER

#include "common_interfaces.h"

// #include <memory>
// #include <string>
// #include <vector>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"

// #include "pluginlib/class_loader.hpp"
// #include "nav2_core/goal_checker.hpp"
// #include "dwb_core/dwb_local_planner.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"


// class MyDWAPlanner
// {
// public:
//     MyDWAPlanner(const std::string & yaml_file, const std::string & map_yaml_file)
//     {
//         // ###################################################
//         // 1. Lifecycle node using YAML parameters
//         // ###################################################
//         rclcpp::NodeOptions options;
//         options.arguments({
//             "--ros-args",
//             "--params-file", yaml_file
//         });

//         node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_dwa_planner", options);


//         // ###################################################
//         // 2. TF buffer + listener
//         // ###################################################
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


//         // ###################################################
//         // 3. Load static map into costmap
//         // ###################################################
//         // TODO: notice !!!
//         // costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
//         //     "local_costmap",
//         //     node_.get(),          // lifecycle or rclcpp node
//         //     "local_costmap",     // parameters namespace
//         //     tf_buffer_
//         // );

//         // lifecycle style configuration
//         costmap_ros_->on_configure(rclcpp_lifecycle::State());


//         // ###################################################
//         // 4. Load DWB planner plugin
//         // ###################################################
//         planner_loader_ =
//             std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
//                 "nav2_controller", "nav2_core::Controller");

//         planner_ = planner_loader_->createSharedInstance("dwb_core::DWBLocalPlanner");

//         planner_->configure(
//             node_,
//             "FollowPath",
//             tf_buffer_,
//             costmap_ros_
//         );

//         planner_->activate();


//         // ###################################################
//         // 5. Load Goal Checker plugin（而不是手写类）
//         // ###################################################
//         goal_checker_loader_ =
//             std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
//                 "nav2_core", "nav2_core::GoalChecker");

//         goal_checker_ = goal_checker_loader_->createSharedInstance("nav2_controller::SimpleGoalChecker");

//         goal_checker_->initialize(node_, "goal_checker", costmap_ros_);


//         RCLCPP_INFO(node_->get_logger(),
//                     "MyDWAPlanner initialized successfully using %s and map %s",
//                      yaml_file.c_str(), map_yaml_file.c_str());
//     }



//     geometry_msgs::msg::TwistStamped compute(
//             const geometry_msgs::msg::PoseStamped & pose,
//             const geometry_msgs::msg::Twist & speed,
//             const nav_msgs::msg::Path & path)
//     {
//         geometry_msgs::msg::TwistStamped cmd;

//         cmd = planner_->computeVelocityCommands(
//             pose,
//             speed,
//             goal_checker_.get()
//         );

//         return cmd;
//     }


// private:
//     rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

//     std::shared_ptr<nav2_core::Controller> planner_;
//     std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> planner_loader_;

//     std::shared_ptr<nav2_core::GoalChecker> goal_checker_;
//     std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> goal_checker_loader_;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// };

// dwa_simple.cpp
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <assert.h>


struct DWAParams {
    // float max_linear_speed = 0.6;      // m/s
    // float min_linear_speed = 0;     // m/s (no backwards)
    // float max_acc = 1.;        // m/s^2

    // float max_yawrate = 1.0;    // rad/s
    // float max_yaw_acc = 1.0;    // rad/s^2

    float dt = 0.1;             // control period for simulation (s)
    float predict_time = 1.0;   // how long to simulate (s)
    float v_res = 0.01;         // linear velocity sampling resolution
    float w_res = 0.01;          // angular velocity sampling resolution

    // scoring weights (positive)
    float weight_heading = 0.7;
    float weight_vel = 0.2;
    float weight_goal_dist = 0.1;

    float dist_tolerance = 0.05; // if within this distance, prefer zero velocity

    float heading_tolerance = 0.1; // if within this distance, prefer zero velocity

};

class DWASimple {
public:
    DWASimple() = default;

    // initialize with parameters (call once)
    void initialize(const DWAParams & params, const MotionConfig& config) {
        // std::cout << "dwa initialize" << std::endl;
        params_ = params;
        motion_config_ = config;
    }

    // compute velocity: inputs are current pose, current velocity (linear, angular), and goal pose
    Pointf<3> computeVelocity(const Pointf<3> &cur_pose, const Pointf<3> &cur_vel, const Pointf<3> &goal) {

        //std::cout << "-- start DWA" << std::endl;

        // if within goal tolerance, stop
        float dist_to_goal = hypot(goal[0] - cur_pose[0], goal[1] - cur_pose[1]);
        float dist_to_heading = 0;//fabs(shortestAngularDistance(cur_pose[2], goal[2]));
        if (dist_to_goal <= params_.dist_tolerance && dist_to_heading <= params_.heading_tolerance) {
            std::cout << "DWA reach goal, dist_to_goal = " << dist_to_goal << ", dist_to_heading = " << dist_to_heading << std::endl; 
            return Pointf<3>{0., 0., 0.};
        }

        // dynamic window based on current vel and accel limits
        float v_min_dyn = std::max(motion_config_.min_v_x, cur_vel[0] + motion_config_.min_a_x * params_.dt);
        float v_max_dyn = std::min(motion_config_.max_v_x, cur_vel[0] + motion_config_.max_a_x * params_.dt);

        //std::cout << "v_max/min = " << v_max_dyn << ", " << v_min_dyn << std::endl;
        assert(v_max_dyn >= v_min_dyn);

        float w_min_dyn = cur_vel[2] + motion_config_.min_a_w * params_.dt;
        float w_max_dyn = cur_vel[2] + motion_config_.max_a_w * params_.dt;

        w_min_dyn = std::max(motion_config_.min_v_w, w_min_dyn);
        w_max_dyn = std::min(motion_config_.max_v_w, w_max_dyn);

        //std::cout << "w_max/min = " << w_max_dyn << ", " << w_min_dyn << std::endl;
        assert(w_max_dyn >= w_min_dyn);

        // sampling
        std::vector<float> v_samples, w_samples;
        for (float v = v_min_dyn; v <= v_max_dyn + 1e-9; v += params_.v_res) { v_samples.push_back(v); }
        for (float w = w_min_dyn; w <= w_max_dyn + 1e-9; w += params_.w_res) { w_samples.push_back(w); }


        if (v_samples.empty()) { v_samples.push_back((v_min_dyn + v_max_dyn) / 2.0); }
        if (w_samples.empty()) { w_samples.push_back((w_min_dyn + w_max_dyn) / 2.0); }

        float best_cost = std::numeric_limits<float>::infinity();
        Pointf<3> best{0., 0., 0.};

        for (const auto& v : v_samples) {
            for (const auto& w : w_samples) {
                //std::cout << "v/w sample = " << v << "/" << w;
                // simulate forward
                Pointf<3> sim = cur_pose;
                float t = 0.0;
                while (t < params_.predict_time - 1e-9) {
                    float step = std::min(params_.dt, params_.predict_time - t);
                    sim[0] += v * cos(sim[2]) * step;
                    sim[1] += v * sin(sim[2]) * step;
                    sim[2] += w * step;
                    t += step;
                }
                //std::cout << ", sim pose = " << sim;
                // costs
                float heading_cost = calcHeadingCost(sim, goal); // 0..pi
                //std::cout << ", heading_cost = " << heading_cost;
                // NEW: velocity cost: prefer larger positive forward speed
                float vel_cost;
                vel_cost = (motion_config_.max_v_x - v) / motion_config_.max_v_x; // 0..1 lower is better 
                if(vel_cost < 0) { vel_cost = 0; }
                //std::cout << ", vel_cost = " << vel_cost;
                float goal_dist = hypot(goal[0] - sim[0], goal[1] - sim[1]);
                //std::cout << ", goal_dist = " << goal_dist;

                // normalize
                float norm_heading = heading_cost / M_PI;
                float norm_vel = std::min(std::max(vel_cost, (float)0.0), (float)2.0); // keep bounded
                float norm_goal = goal_dist / (goal_dist + 1.0);

                float cost = params_.weight_heading * norm_heading
                            + params_.weight_vel * norm_vel
                            + params_.weight_goal_dist * norm_goal;

                //std::cout << ", normalize_cost = " << cost;
                
                // cost 越低约好
                if (cost + 1e-9 < best_cost) {
                    best_cost = cost;
                    best[0] = v;
                    best[2] = w;
                } else if (std::fabs(cost - best_cost) < 1e-9) {
                    // tie-breaker: prefer larger forward velocity (not abs)
                    if (v > best[0]) {
                        best[1] = v;
                        best[2] = w;
                    }
                }
                //std::cout << ", v/w = " << v << "/" << w << std::endl;
            }
        }

        return best;
    }

    MotionConfig motion_config_;

private:
    DWAParams params_;

    // heading cost: angle between robot's forward direction at sim pose and vector to goal
    static float calcHeadingCost(const Pointf<3> &sim, const Pointf<3> &goal) {
        float dx = goal[0] - sim[0];
        float dy = goal[1] - sim[1];
        float goal_yaw = std::atan2(dy, dx);
        float d = shortestAngularDistance(goal_yaw, sim[2]);
        return std::fabs(d); // 0..pi
    }
};


#endif