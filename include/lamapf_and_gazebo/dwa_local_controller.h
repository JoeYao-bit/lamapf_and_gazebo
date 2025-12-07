#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pluginlib/class_loader.hpp"
#include "nav2_core/goal_checker.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class MyDWAPlanner
{
public:
    MyDWAPlanner(const std::string & yaml_file, const std::string & map_yaml_file)
    {
        // ###################################################
        // 1. Lifecycle node using YAML parameters
        // ###################################################
        rclcpp::NodeOptions options;
        options.arguments({
            "--ros-args",
            "--params-file", yaml_file
        });

        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_dwa_planner", options);


        // ###################################################
        // 2. TF buffer + listener
        // ###################################################
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // ###################################################
        // 3. Load static map into costmap
        // ###################################################
        // TODO: notice !!!
        // costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        //     "local_costmap",
        //     node_.get(),          // lifecycle or rclcpp node
        //     "local_costmap",     // parameters namespace
        //     tf_buffer_
        // );

        // lifecycle style configuration
        costmap_ros_->on_configure(rclcpp_lifecycle::State());


        // ###################################################
        // 4. Load DWB planner plugin
        // ###################################################
        planner_loader_ =
            std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
                "nav2_controller", "nav2_core::Controller");

        planner_ = planner_loader_->createSharedInstance("dwb_core::DWBLocalPlanner");

        planner_->configure(
            node_,
            "FollowPath",
            tf_buffer_,
            costmap_ros_
        );

        planner_->activate();


        // ###################################################
        // 5. Load Goal Checker plugin（而不是手写类）
        // ###################################################
        goal_checker_loader_ =
            std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
                "nav2_core", "nav2_core::GoalChecker");

        goal_checker_ = goal_checker_loader_->createSharedInstance("nav2_controller::SimpleGoalChecker");

        goal_checker_->initialize(node_, "goal_checker", costmap_ros_);


        RCLCPP_INFO(node_->get_logger(),
                    "MyDWAPlanner initialized successfully using %s and map %s",
                     yaml_file.c_str(), map_yaml_file.c_str());
    }



    geometry_msgs::msg::TwistStamped compute(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & speed,
            const nav_msgs::msg::Path & path)
    {
        geometry_msgs::msg::TwistStamped cmd;

        cmd = planner_->computeVelocityCommands(
            pose,
            speed,
            goal_checker_.get()
        );

        return cmd;
    }


private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    std::shared_ptr<nav2_core::Controller> planner_;
    std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> planner_loader_;

    std::shared_ptr<nav2_core::GoalChecker> goal_checker_;
    std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> goal_checker_loader_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

