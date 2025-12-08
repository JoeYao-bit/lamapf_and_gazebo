#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

// control single agent move to its target

class GoalListener : public rclcpp::Node
{
public:

    GoalListener(int agent_id, std::string goal_pub_topic) : Node("goal_listener")
    {
        agent_id_ = agent_id;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // pub goal into local controller
        goal_pub_ = this->create_publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>(goal_pub_topic, 10);

        // goal_pose receive rviz's Nav Goal
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot0/goal_pose", 10,
            std::bind(&GoalListener::goalCallback, this, std::placeholders::_1));
           
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {

        });    
    }


    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

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

        goal_x_ = x;
        goal_y_ = y;
        goal_theta_ = yaw;
        
        // get start from tf tree
        try {
            // 获取最新 transform，阻塞最多2秒
            auto transformStamped = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero,
                std::chrono::seconds(2));

            x = transformStamped.transform.translation.x;

            y = transformStamped.transform.translation.y;

            // geometry_msgs::Quaternion -> tf2::Quaternion
            tf2::Quaternion q;
            tf2::fromMsg(transformStamped.transform.rotation, q);

            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                        "Current pose: x=%.3f, y=%.3f, theta=%.3f rad",
                        x, y, yaw);

            start_x_     = x;
            start_y_     = y;
            start_theta_ = yaw;

        }
        catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for transform map -> base_footprint: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        // pub goal to local controller

        lamapf_and_gazebo_msgs::msg::UpdateGoal goal_msg;
        goal_msg.start_x   = start_x_;
        goal_msg.start_y   = start_y_;
        goal_msg.start_yaw = start_theta_;

        goal_msg.target_x   = goal_x_;
        goal_msg.target_y   = goal_y_;
        goal_msg.target_yaw = goal_theta_;

        goal_msg.agent_id   = agent_id_;
        goal_msg.wait       = false;

        std::stringstream ss2;
        ss2 << "pub goal start = (" << start_x_ << ", " << start_y_ << ", " << start_theta_ << "), ";
        ss2 << "target = (" << goal_x_ << ", " << goal_y_ << ", " << goal_theta_ << "), ";
        ss2 << "agent id = " << agent_id_;

        RCLCPP_INFO(this->get_logger(), ss2.str().c_str());

        goal_pub_->publish(goal_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<lamapf_and_gazebo_msgs::msg::UpdateGoal>::SharedPtr goal_pub_;

    float goal_x_,  goal_y_,  goal_theta_;

    float start_x_, start_y_, start_theta_;

    int agent_id_;

};

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);

    // 创建初始控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowControllerDWA>(MotionConfig());
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

    auto goal_listener_node = std::make_shared<GoalListener>(0, "/robot0/local_goal");                                     

    executor.add_node(goal_listener_node);

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();
    
    return 0;
}