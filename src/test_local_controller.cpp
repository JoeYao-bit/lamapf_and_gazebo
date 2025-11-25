#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

// control single agent move to its target


int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    
    // receive localization 
    // 由于/amcl_pose无法通过配置定时发布，仅在更新时发布，因此通过tf获取定位信息

    auto node = rclcpp::Node::make_shared("get_initial_pose");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    geometry_msgs::msg::TransformStamped transformStamped;
    bool got_transform = false;

    RCLCPP_INFO(node->get_logger(), "Waiting for transform map -> base_footprint...");

    while (rclcpp::ok() && !got_transform) {
        try {
            // 获取最新 transform，阻塞等待最多 2 秒
            transformStamped = tf_buffer->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero,
                std::chrono::seconds(2));

            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(node->get_logger(),
                        "Pose in map: x=%.3f, y=%.3f, theta=%.3f rad",
                        x, y, yaw);

            got_transform = true; // 成功获取后退出循环
        }
        catch (tf2::TransformException & ex) {
            RCLCPP_WARN(node->get_logger(),
                        "Waiting for transform map -> base_footprint: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    return 0;
    
    // 创建初始控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowController>(MotionConfig());
    auto rot_ctl  = std::make_shared<ConstantRotateController>(MotionConfig());

    // create circle agent represent turtlebot
    auto agent = std::make_shared<CircleAgent<2> >(0.2, 0, dim);

    // create local control node

    double time_interval = 0.1;//.1; // second

    auto agent_control_node = std::make_shared<LocalController>(agent, line_ctl, rot_ctl, 1, time_interval,
                                                                "/goal",
                                                                "/scan",
                                                                "/commands/velocity");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    executor.add_node(agent_control_node);

    return 0;
}