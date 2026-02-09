#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

// control single agent move to its target

class OneShotStartListener : public rclcpp::Node
{
public:
    OneShotStartListener(
        const std::string & node_name,
        const std::string & map_name,
        const std::string & frame_name)
    : Node(node_name),
      map_name_(map_name),
      frame_name_(frame_name)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // ★ CHANGED: 创建 timer，在 timer 里查 TF
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&OneShotStartListener::tryLookupTF, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "Listening TF from [%s] to [%s]",
                    map_name_.c_str(), frame_name_.c_str());
    }

    bool startReceived() const { return got_start_; }

    Pointf<3> start_;

    std::string map_name_, frame_name_;

private:

    // =========================
    // subscription callback
    // =========================
    void startAndTargetCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/)
    {
        // ★ CHANGED: callback 里什么都不等
        got_start_trigger_ = true;
    }

    // =========================
    // timer callback
    // =========================
    void tryLookupTF()
    {
        // 还没收到 start 触发，不做任何事
        if (got_start_) {
            return;
        }

        // 非阻塞检查
        if (!tf_buffer_->canTransform(
                map_name_, frame_name_, tf2::TimePointZero)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for TF %s -> %s",
                map_name_.c_str(), frame_name_.c_str());
            return;
        }

        try {
            auto transformStamped = tf_buffer_->lookupTransform(
                map_name_, frame_name_, tf2::TimePointZero);

            start_[0] = transformStamped.transform.translation.x;
            start_[1] = transformStamped.transform.translation.y;

            tf2::Quaternion q;
            tf2::fromMsg(transformStamped.transform.rotation, q);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            start_[2] = yaw;

            RCLCPP_INFO(this->get_logger(),
                        "Get start from TF: x=%.3f, y=%.3f, theta=%.3f rad",
                        start_[0], start_[1], start_[2]);

            got_start_ = true;

            // ★ CHANGED: 一次性任务，成功后停掉 timer
            timer_->cancel();
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                        "TF exception: %s", ex.what());
        }
    }

private:
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ★ CHANGED: timer
    rclcpp::TimerBase::SharedPtr timer_;

    // ★ CHANGED: 两阶段状态
    bool got_start_trigger_ = false;
    bool got_start_ = false;
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
//std::string map_name = "/home/wangweilab/ros2_ws/src/lamapf_and_gazebo/map/my_map_2602052038_plan";
std::string map_name = "/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/map/my_map_2602052038_plan";

PictureLoader loader_local(map_name+".pgm", is_grid_occupied_pgm);
std::string yaml_file_path = map_name+".yaml";

DimensionLength* dim_local = loader_local.getDimensionInfo();

auto is_occupied_local = [](const Pointi<2> & pt) -> bool { return loader_local.isOccupied(pt); };

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

std::vector<Pointf<3> > loadTargets(std::string data_path) {
    
    std::ifstream fin(data_path);
    std::vector<Pointf<3> > data;
    if (!fin.is_open()) {
        std::cerr << "Failed to open file " << data_path << std::endl;
        return {};
    }
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;            // 跳过空行
        if (line[0] == '#') continue;           // 跳过注释行

        std::istringstream iss(line);
        Pointf<3> ptf;

        if (!(iss >> ptf[0] >> ptf[1] >> ptf[2])) {
            std::cerr << "Parse error: " << line << std::endl;
            continue;
        } else {
            std::cout << "line " << ": ptf = " << ptf << std::endl;
        }

        data.push_back(ptf);
    }
    fin.close();
    return data;
}

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);

    int agent_id = 0;
    if (argc > 1) {
        agent_id = std::stoi(argv[1]);
    }

    // set config for using of PtfToGridPicOnly
    dim[0] = dim_local[0];
    
    dim[1] = dim_local[1];

    reso = 0.05;

    //return 0; // AC till here
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);

    // construct instance
    // create circle agent represent turtlebot
    double time_interval = 0.1;// s

    std::pair<AgentPtrs<2>, InstanceOrients<2> > instances = 
        {}; // get all instances


    float ox,oy,otheta;
    getOriginFromYAMLFile(yaml_file_path, ox, oy, otheta);

    float reso_local;
    getResolutionFromYAMLFile(yaml_file_path, reso_local);

    std::cout << "resolution of map = " << reso_local << std::endl;


    // TODO: load start from text file
    std::vector<Pointf<3> > targets = loadTargets(std::string("/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/config/target_config.txt"));

    //return 0;

    // how much targets determine how much start need to get
    int num_of_agent = targets.size();

    for(int agent=0; agent<num_of_agent; agent++) {

        std::stringstream ss;

        ss << "robot" << agent << "/base_footprint";

        //ss << "/robot" << agent << "/goal_pose";


        auto node = std::make_shared<OneShotStartListener>("one_shot_start_listener", std::string("map"), ss.str());

        RCLCPP_INFO(node->get_logger(), "Waiting for start from tf...");

        // // 循环等待直到收到一次消息
        // while (rclcpp::ok() && !node->startReceived()) {
        //     rclcpp::spin_some(node);
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));
        // }

        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node);

        while (rclcpp::ok() && !node->startReceived()) {
            exec.spin_once(std::chrono::milliseconds(100));
        }

        // 转换到地图坐标系下，参考地图yaml文件中的原点和姿态
        Pose<int, 2> start_pose, target_pose;

        worldToPixelYAML(node->start_[0], node->start_[1], ox, oy, otheta, reso_local, dim_local[1], start_pose.pt_[0],  start_pose.pt_[1]);

        worldToPixelYAML(targets[agent][0], targets[agent][1], ox, oy, otheta, reso_local, dim_local[1], target_pose.pt_[0],  target_pose.pt_[1]);

        start_pose.orient_  = radiusToOrient(worldYawToPixelYaw(node->start_[2], otheta));

        target_pose.orient_ = radiusToOrient(worldYawToPixelYaw(targets[agent][2], otheta));

        std::cout << "raw/new start  yaw = " << node->start_[2] << "/" << pixelYawToWorldYaw(orientToRadius(start_pose.orient_), otheta) << std::endl;

        auto agent_ptr = std::make_shared<CircleAgent<2> >(0.2, agent, dim_local);

        instances.first.push_back(agent_ptr);    

        instances.second.push_back({start_pose, target_pose});    

        

    }

    //return 0; // AC till here

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

    std::thread t1([&]() { executor.spin(); });
    
    t1.join();
    
    return 0;
}