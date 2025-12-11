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
    OneShotGoalListener(std::string node_name) : Node(node_name)
    {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot0/goal_pose", 10,
            std::bind(&OneShotGoalListener::goalCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    }

    bool goalReceived() const { return got_goal_; }

    Pointf<3> goal_, start_;

private:

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        goal_[0] = msg->pose.position.x;
        goal_[1] = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
                    "Received one-time goal: x=%.3f, y=%.3f, theta=%.3f rad",
                    goal_[0], goal_[1], yaw);

        goal_[2] = yaw;

        // get start from tf tree
        try {
            // 获取最新 transform，阻塞最多2秒
            auto transformStamped = tf_buffer_->lookupTransform(
                "map", "robot0/base_footprint", tf2::TimePointZero,
                std::chrono::seconds(2));

            start_[0] = transformStamped.transform.translation.x;

            start_[1] = transformStamped.transform.translation.y;

            // geometry_msgs::Quaternion -> tf2::Quaternion
            tf2::Quaternion q2;
            tf2::fromMsg(transformStamped.transform.rotation, q2);

            tf2::Matrix3x3(q2).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                        "Get start from current pose: x=%.3f, y=%.3f, theta=%.3f rad",
                        start_[0], start_[1], yaw);

            start_[2] = yaw;

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


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    bool got_goal_ = false;

};

std::shared_ptr<ActionDependencyGraph<2> > CenteralController::ADG_ = nullptr;

bool CenteralController::paused_ = true;

std::vector<int> CenteralController::progress_of_agents_ = {};  

std::vector<std::shared_ptr<Pose<int, 2>> > CenteralController::all_poses_ = {};

Pointfs<3> CenteralController::all_agent_poses_ = {};

std::pair<AgentPtrs<2>, InstanceOrients<2> > CenteralController::instances_ = {};

DimensionLength* CenteralController::dim_ = nullptr;

std::vector<std::pair<Pointf<3>, Pointf<3> > > CenteralController::recover_tasks_ = {};  

bool CenteralController::pub_init_target_ = true;

bool CenteralController::need_replan_ = false;



// load map
PictureLoader loader_local("/home/wangweilab/my_map.pgm", is_grid_occupied_pgm);
std::string yaml_file_path = "/home/wangweilab/my_map.yaml";

DimensionLength* dim_local = loader_local.getDimensionInfo();

auto is_occupied_local = [](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };


void StartAndTargetVisualize() {
    //
}


std::shared_ptr<ActionDependencyGraph<2> > CenteralControllerNew::ADG_ = nullptr;

bool CenteralControllerNew::paused_ = true;

Pointfs<3> CenteralControllerNew::all_agent_poses_ = {};

std::vector<int> CenteralControllerNew::progress_of_agents_ = {};  

std::vector<std::shared_ptr<Pose<int, 2>> > CenteralControllerNew::all_poses_ = {};

std::pair<AgentPtrs<2>, InstanceOrients<2> > CenteralControllerNew::instances_ = {};

DimensionLength* CenteralControllerNew::dim_ = nullptr;

bool CenteralControllerNew::pub_init_target_ = true;

bool CenteralControllerNew::need_replan_ = false;

std::vector<bool> CenteralControllerNew::agent_finishes_ = {};

POSE_TO_PTF_FUNC CenteralControllerNew::pose_to_ptf_func_;

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);

    // set config for using of PtfToGridPicOnly
    dim[0] = dim_local[0];
    
    dim[1] = dim_local[1];

    reso = 0.05;

    auto node = std::make_shared<OneShotGoalListener>("one_shot_goal_listener");

    RCLCPP_INFO(node->get_logger(), "Waiting for one-time 2D Nav Goal from RViz2...");

    // 循环等待直到收到一次消息
    while (rclcpp::ok() && !node->goalReceived()) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    //return 0; // AC till here
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    // construct instance
    // create circle agent represent turtlebot
    auto agent_ptr = std::make_shared<CircleAgent<2> >(0.2, 0, dim_local);
    double time_interval = 0.1;// s

    std::pair<AgentPtrs<2>, InstanceOrients<2> > instances = 
        {}; // get all instances

    instances.first.push_back(agent_ptr);    

    float ox,oy,otheta;
    getOriginFromYAMLFile(yaml_file_path, ox, oy, otheta);

    float reso_local;
    getResolutionFromYAMLFile(yaml_file_path, reso_local);

    std::cout << "resolution of map = " << reso_local << std::endl;

    // 转换到地图坐标系下，参考地图yaml文件中的原点和姿态
    Pose<int, 2> start_pose, target_pose;

    worldToPixelYAML(node->start_[0], node->start_[1], ox, oy, otheta, reso_local, dim_local[1], start_pose.pt_[0],  start_pose.pt_[1]);

    worldToPixelYAML(node->goal_[0],  node->goal_[1],  ox, oy, otheta, reso_local, dim_local[1], target_pose.pt_[0], target_pose.pt_[1]);

    start_pose.orient_  = radiusToOrient(worldYawToPixelYaw(node->start_[2], otheta));

    target_pose.orient_ = radiusToOrient(worldYawToPixelYaw(node->goal_[2], otheta));

    std::cout << "raw/new start  yaw = " << node->start_[2] << "/" << pixelYawToWorldYaw(orientToRadius(start_pose.orient_), otheta) << std::endl;

    std::cout << "raw/new target yaw = " << node->goal_[2] << "/" << pixelYawToWorldYaw(orientToRadius(target_pose.orient_), otheta) << std::endl;

    instances.second.push_back({start_pose, target_pose});    

    // start_pose and target_pose is right
    // {
    //     Canvas canvas("LA-MAPF visualization", dim_local[0], dim_local[1], 1./reso, 5);
    //     canvas.resolution_ = 1./reso;
    //     while(rclcpp::ok()) {
    //         canvas.resetCanvas();
    //         canvas.drawEmptyGrid();
    //         canvas.drawGridMap(dim_local, is_occupied_local);

    //         agent_ptr->drawOnCanvas(start_pose, canvas, COLOR_TABLE[0], false);

    //         agent_ptr->drawOnCanvas(target_pose, canvas, COLOR_TABLE[1], false);

    //         char key = canvas.show();
    //     }
    //     return 0;
    // }



    POSE_TO_PTF_FUNC ptpfunc = [ox, oy, otheta, reso_local, dim_local](const Pose<int, 2>& pose) -> Pointf<3> {
        Pointf<3> retv;
        pixelToWorldYAML(pose.pt_[0], pose.pt_[1], ox, oy, otheta, reso_local, dim_local[1], retv[0], retv[1]);
        retv[2] = pixelYawToWorldYaw(orientToRadius(pose.orient_), otheta);
        return retv;

    };

    std::cout << "otheta = " << otheta << std::endl;
    
    std::cout << "orientToRadius(3) = " << orientToRadius(3) << std::endl;

    std::cout << "pixelYawToWorldYaw(orientToRadius(3), otheta) = " <<  pixelYawToWorldYaw(orientToRadius(3), otheta) << std::endl;


    
    // need test
    // 启动中央控制器，路径可视化
    // start central controller
    auto central_controller = std::make_shared<CenteralControllerNew>(dim_local, is_occupied_local, instances, 
                                                                   ptpfunc,
                                                                   time_interval, 
                                                                   true); // enable opencv window



    executor.add_node(central_controller);

    // 创建局部控制器

    auto line_ctl = std::make_shared<TwoPhaseLineFollowControllerReal>(MotionConfig());
    auto rot_ctl  = std::make_shared<ConstantRotateController>(MotionConfig());

    // create local control node

    auto agent_control_node = std::make_shared<LocalController>(agent_ptr, line_ctl, 1, time_interval,
                                                                "/robot0/amcl_pose",
                                                                "/robot0/local_goal",
                                                                "/robot0/scan",
                                                                "/robot0/commands/velocity");  
                                                        

    executor.add_node(agent_control_node);

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();
    
    return 0;
}