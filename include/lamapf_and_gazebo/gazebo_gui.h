#ifndef GAZEBO_GUI
#define GAZEBO_GUI

#include "common_interfaces.h"


class GazeboGUI : public rclcpp::Node {
public:

    GazeboGUI(std::shared_ptr<CenteralController> ctl, float time_interval = 0.1) 
        : ctl_(ctl), rclcpp::Node("GazeboGUI") {
        assert(ctl_ != nullptr);
        gazeboInitialize();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000*time_interval)), [this]() {
            //std::stringstream ss;
            //ss << "during CentralController loop, paused = " << paused_;
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            updateAllAgentPoseInGazebo();
        });
    }

    void gazeboInitialize() {
        //std::stringstream ss;
        //ss << "during CentralController loop, paused = " << paused_;
        RCLCPP_INFO(this->get_logger(), "start gazeboInitialize");
        set_pose_clinet = std::make_shared<EntityPoseSetter>();
        entity_pawner_clinet = std::make_shared<EntitySpawner>();

        // add instance to gazebo
        geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = 0.0;  // 设置模型初始位置
        initial_pose.position.y = 0.0;
        initial_pose.position.z = 0.5;
        initial_pose.orientation.w = 1.0;
        RCLCPP_INFO(this->get_logger(), "flag 1");

        // add agent to gazebo
        for (int id=0; id < ctl_->instances_.first.size(); id++) {
            const auto& agent = ctl_->instances_.first[id];
            const auto& start_pt = ctl_->instances_.second[id].first.pt_; 
            auto color = COLOR_TABLE[id%30];
            double start_theta = orientToRadius(ctl_->instances_.second[id].first.orient_);

            Pointf<3> ptf = GridToPtfPicOnly(start_pt);

            initial_pose.position.x = ptf[0];  // transfer to world coordinate system
            initial_pose.position.y = ptf[1];
            initial_pose.position.z = agent_height;

            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, start_theta); // Create this quaternion from roll/pitch/yaw (in radians)

            initial_pose.orientation.x = orientation.x();
            initial_pose.orientation.y = orientation.y();
            initial_pose.orientation.z = orientation.z();
            initial_pose.orientation.w = orientation.w();

            // do not spawn circle or block, but spawn real robots
            std::string file_path3 = ROBOT_SDFS[REAL_ROBOTS[id]];
            if(agent->type_ == "Circle") {
                spawnAgentGazebo(file_path3, std::string("Circle_")+std::to_string(agent->id_), initial_pose, entity_pawner_clinet);
            } else if(agent->type_ == "Block_2D") {
                spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, entity_pawner_clinet);
            } else {
                RCLCPP_INFO(this->get_logger(), "undefined agent type");
                std::exit(0);
            }
        }

        RCLCPP_INFO(this->get_logger(), "finish gazeboInitialize");

    }


    // set all poses of agent in gazebo to allAgentPoses
    void updateAllAgentPoseInGazebo() {

        const auto& allAgentPoses = ctl_->all_agent_poses_; 
        const auto& agents        = ctl_->instances_.first;

        for (int id=0; id<agents.size(); id++) {
            const auto& agent = agents[id];

            double target_x = allAgentPoses[id][0],
                   target_y = allAgentPoses[id][1],
                   target_z = agent_height,
                   target_theta = allAgentPoses[id][2];

            if(agent->type_ == "Circle") {

                setModelPose(std::string("Circle_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet);

            } else if(agent->type_ == "Block_2D") {
            
                setModelPose(std::string("Block_2D_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet);

            }
        }
    }
    
    std::shared_ptr<CenteralController> ctl_;

    // set gazebo agent pose
    EntityPoseSetterPtr set_pose_clinet = nullptr;
    // spawn entity in gazebo
    EntitySpawnerPtr entity_pawner_clinet = nullptr;

    rclcpp::TimerBase::SharedPtr timer_;

};


#endif