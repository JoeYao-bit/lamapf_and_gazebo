#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

// control single agent move to its target

class OneShotPoseListener : public rclcpp::Node
{
public:
    OneShotPoseListener() : Node("one_shot_pose_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        got_pose_ = false;
    }

    bool poseReceived() const { return got_pose_; }

    void spinUntilPose()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform map -> base_footprint...");
        while (rclcpp::ok() && !got_pose_) {
            try {
                // 获取最新 transform，阻塞最多2秒
                auto transformStamped = tf_buffer_->lookupTransform(
                    "map", "base_footprint", tf2::TimePointZero,
                    std::chrono::seconds(2));

                double x = transformStamped.transform.translation.x;
                double y = transformStamped.transform.translation.y;

                // geometry_msgs::Quaternion -> tf2::Quaternion
                tf2::Quaternion q;
                tf2::fromMsg(transformStamped.transform.rotation, q);

                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                RCLCPP_INFO(this->get_logger(),
                            "Current pose: x=%.3f, y=%.3f, theta=%.3f rad",
                            x, y, yaw);
                x_ = x;
                y_ = y;
                theta_ = yaw;

                got_pose_ = true;  // 获取一次后退出循环
            }
            catch (tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(),
                            "Waiting for transform map -> base_footprint: %s", ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }

            rclcpp::spin_some(this->shared_from_this());
        }
    }

    float x_, y_, theta_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool got_pose_;
};

class OneShotGoalListener : public rclcpp::Node
{
public:
    OneShotGoalListener() : Node("one_shot_goal_listener")
    {
        got_goal_ = false;
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&OneShotGoalListener::goalCallback, this, std::placeholders::_1));
    }

    bool goalReceived() const { return got_goal_; }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (got_goal_) return; // 已获取一次就返回

        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
                    "Received one-time goal: x=%.3f, y=%.3f, theta=%.3f rad",
                    x, y, yaw);
        x_ = x;
        y_ = y;
        theta_ = yaw;
        got_goal_ = true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    bool got_goal_;

    float x_, y_, theta_;
};

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<OneShotPoseListener>();

    node1->spinUntilPose();

    // 接收 /goal_pose，rviz2上设置的终点坐标
    auto node2 = std::make_shared<OneShotGoalListener>();

    RCLCPP_INFO(node2->get_logger(), "Waiting for one-time 2D Nav Goal from RViz2...");

    // 循环等待直到收到一次消息
    while (rclcpp::ok() && !node2->goalReceived()) {
        rclcpp::spin_some(node2);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    //return 0;

    // 创建初始控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowController>(MotionConfig());
    auto rot_ctl  = std::make_shared<ConstantRotateController>(MotionConfig());

    // create circle agent represent turtlebot
    auto agent = std::make_shared<CircleAgent<2> >(0.2, 0, dim);

    // create local control node

    double time_interval = 0.1;//.1; // second

    auto agent_control_node = std::make_shared<LocalController>(agent, line_ctl, rot_ctl, 1, time_interval,
                                                                "/amcl_pose",
                                                                "/goal",
                                                                "/scan",
                                                                "/commands/velocity");

    // set start and goal state
    
    agent_control_node->start_ptf_[0]  = node1->x_;    
    agent_control_node->start_ptf_[1]  = node1->y_;    
    agent_control_node->start_ptf_[2]  = node1->theta_;    
    agent_control_node->target_ptf_[0] = node2->x_;    
    agent_control_node->target_ptf_[1] = node2->y_;    
    agent_control_node->target_ptf_[2] = node2->theta_;  
    agent_control_node->wait_          = false;

    agent_control_node->line_ctl_->finish_rotate_ = false;
    agent_control_node->line_ctl_->finish_move_ = false;
    
    agent_control_node->line_ctl_->pt1_ = agent_control_node->start_ptf_;
    agent_control_node->line_ctl_->pt2_ = agent_control_node->target_ptf_;

    std::stringstream ss2;
    ss2 << "in LocalController, set external goal, agent_" << agent_control_node->agent_->id_ << " ";
    ss2 << agent_control_node->start_ptf_ << "->" << agent_control_node->target_ptf_;
    ss2 << ", wait = " << agent_control_node->wait_;
    RCLCPP_INFO(node2->get_logger(), ss2.str().c_str());

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    executor.add_node(agent_control_node);

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();
    
    return 0;
}