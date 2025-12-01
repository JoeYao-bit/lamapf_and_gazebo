#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

// control single agent move to its target

class OneShotGoalListener : public rclcpp::Node
{
public:
    OneShotGoalListener() : Node("one_shot_goal_listener")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot0/goal_pose", 10,
            std::bind(&OneShotGoalListener::goalCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    bool goalReceived() const { return got_goal_; }

private:

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
                    "Received one-time goal: x=%.3f, y=%.3f, theta=%.3f rad",
                    goal_x_, goal_y_, yaw);

        goal_theta_ = yaw;

        // get start from tf tree
        try {
            // 获取最新 transform，阻塞最多2秒
            auto transformStamped = tf_buffer_->lookupTransform(
                "map", "/robot0/base_footprint", tf2::TimePointZero,
                std::chrono::seconds(2));

            start_x_ = transformStamped.transform.translation.x;

            start_y_ = transformStamped.transform.translation.y;

            // geometry_msgs::Quaternion -> tf2::Quaternion
            tf2::Quaternion q2;
            tf2::fromMsg(transformStamped.transform.rotation, q2);

            tf2::Matrix3x3(q2).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                        "Get start from current pose: x=%.3f, y=%.3f, theta=%.3f rad",
                        start_x_, start_y_, yaw);

            start_theta_ = yaw;

        }
        catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for transform map -> base_footprint: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        got_goal_ = true;
    
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    float goal_x_,  goal_y_,  goal_theta_;

    float start_x_, start_y_, start_theta_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    bool got_goal_ = false;

};


int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OneShotGoalListener>();

    RCLCPP_INFO(node->get_logger(), "Waiting for one-time 2D Nav Goal from RViz2...");

    // 循环等待直到收到一次消息
    while (rclcpp::ok() && !node->goalReceived()) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;

    // TODO: 启动中央控制器，路径可视化

    // 创建局部控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowControllerReal>(MotionConfig());
    auto rot_ctl  = std::make_shared<ConstantRotateController>(MotionConfig());

    // create circle agent represent turtlebot
    auto agent = std::make_shared<CircleAgent<2> >(0.2, 0, dim);

    // create local control node

    double time_interval = 0.1;//.1; // second

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    auto agent_control_node = std::make_shared<LocalController>(agent, line_ctl, 1, time_interval,
                                                                "/robot0/amcl_pose",
                                                                "/robot0/local_goal",
                                                                "/robot0/scan",
                                                                "/robot0/commands/velocity");  
                                                        

    executor.add_node(agent_control_node);

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();
    
    return 0;
}