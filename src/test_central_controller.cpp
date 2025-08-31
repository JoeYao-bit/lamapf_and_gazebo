#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"

#include "lamapf_and_gazebo/gazebo_gui.h"

#include <gtest/gtest.h>

#include "lamapf_and_gazebo/fake_agents.h"

#include "lamapf_and_gazebo/common_interfaces.h"

void startCentralControllerNode(const std::string& file_path) {
    //
}

// void startSingleRobotThread(const AgentPtr<2>& agent,
//                             const LineFollowControllerPtr& line_ctl,
//                             const RotateControllerPtr& rot_ctl) {

//     std::cout << "flag 0.1" << std::endl;
//     // 在不同的线程中运行节点
//     std::thread publisher_thread([&]() {
//         std::cout << "flag 0.2" << std::endl;
//         // 每个node需要一个独立进程以及context
//         auto context = std::make_shared<rclcpp::Context>();
//         context->init(0, nullptr);
//         rclcpp::NodeOptions options = rclcpp::NodeOptions().context(context);
//         std::cout << "flag 0.3" << std::endl;
//         // 创建节点
//         auto agent_node_thread = std::make_shared<LocalController>(agent, line_ctl, rot_ctl, options, 1);
//         rclcpp::spin(agent_node_thread);
//     });


// } 


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(int id) : Node((std::string("publish_node_")+std::to_string(id)).c_str()),id_(id)
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() {
        std::stringstream ss;
        ss << id_ << " timer callback running";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;  // ✅ 如果是局部变量，构造完就析构
  int id_;
};


// void startSingleRobotNodes(const std::vector<AgentPtr<2>>& agents,
//                            const std::vector<LineFollowControllerPtr>& line_ctls,
//                            const std::vector<RotateControllerPtr>& rot_ctls) {
//     assert(agents.size() == line_ctls.size());
//     assert(agents.size() == rot_ctls.size());

//     std::cout << "start " << agents.size() << " agent nodes" << std::endl;
//     // 使用 MultiThreadedExecutor 或 StaticSingleThreadedExecutor，
//     // 把多个节点交给一个 Executor 管理，Executor 内部会调度多线程。
//     rclcpp::executors::MultiThreadedExecutor executor;
//     // 我发现executor.add_node放在循环中，比如for循化，那么time_中的内容不会运行，但如果一个个手动添加，则会运行，这是为什么
//     // executor.add_node() 只保存了节点的 裸指针（在内部包装成 weak_ptr）
//     // 如果你在循环中创建的节点没有被其他变量持有，它会在循环迭代结束时析构
//     // 析构后，Timer 对象也被销毁，回调自然不会触发
//     // 外部保存，即可避免这一缺陷。

//     std::vector<std::shared_ptr<LocalController> > nodes;

//     for(int i=0; i<agents.size(); i++) {
//         auto agent_node = std::make_shared<LocalController>(agents[i], line_ctls[i], rot_ctls[i], 1);
//         executor.add_node(agent_node);
//         nodes.push_back(agent_node); 
//     }
//     executor.spin();
// }

std::shared_ptr<ActionDependencyGraph<2> > CenteralController::ADG_ = nullptr;

bool CenteralController::paused_ = true;

std::vector<int> CenteralController::progress_of_agents_ = {};  

std::vector<std::shared_ptr<Pose<int, 2>> > CenteralController::all_poses_ = {};

Pointfs<3> CenteralController::all_agent_poses_ = {};

std::pair<AgentPtrs<2>, InstanceOrients<2> > CenteralController::instances_ = {};

DimensionLength* CenteralController::dim_ = nullptr;

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);

    // load instances
    std::string file_path = map_test_config.at("la_ins_path");

    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load instances from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load instances from path " << file_path << " failed" << std::endl;
        return 1;
    }
    std::stringstream ss;
    ss << "map scale = " << dim[0] << "*" << dim[1];
    std::cout << ss.str() << std::endl;

    std::pair<AgentPtrs<2>, InstanceOrients<2> > instances = 
        deserializer.getTestInstance({5}, 1).front(); // get all instances

    std::vector<LineFollowControllerPtr> line_ctls(instances.first.size(), std::make_shared<ConstantLineFollowController>(MotionConfig()));
    std::vector<RotateControllerPtr> rot_ctls(instances.first.size(), std::make_shared<ConstantRotateController>(MotionConfig()));

    const auto& agents = instances.first;
    // start single robot controller
    // finish but need more test
    std::cout << "start " << agents.size() << " agent nodes" << std::endl;
    // 使用 MultiThreadedExecutor 或 StaticSingleThreadedExecutor，
    // 把多个节点交给一个 Executor 管理，Executor 内部会调度多线程。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), instances.first.size() + 3);
    // 我发现executor.add_node放在循环中，比如for循化，那么time_中的内容不会运行，但如果一个个手动添加，则会运行，这是为什么
    // executor.add_node() 只保存了节点的 裸指针（在内部包装成 weak_ptr）
    // 如果你在循环中创建的节点没有被其他变量持有，它会在循环迭代结束时析构
    // 析构后，Timer 对象也被销毁，回调自然不会触发
    // 外部保存，即可避免这一缺陷。

    double time_interval = 0.1;//.1; // second

    std::vector<std::shared_ptr<LocalController> > nodes;
    for(int i=0; i<agents.size(); i++) {
        std::cout << "i = " << i << ", " << instances.second[i].first << std::endl;
        PosePtr<int, 2> start_pose = std::make_shared<Pose<int, 2> >(instances.second[i].first);
        Pointf<3> init_pose = PoseIntToPtf(start_pose);  
        std::cout << "init_pose = " << init_pose << std::endl;
        auto agent_node = std::make_shared<LocalController>(agents[i], line_ctls[i], rot_ctls[i], init_pose, instances.second.size(), time_interval);
        executor.add_node(agent_node);
        nodes.push_back(agent_node); 
    }
    std::thread t1([&]() { executor.spin(); });

    rclcpp::executors::MultiThreadedExecutor executor2;
    // start central controller
    auto central_controller = std::make_shared<CenteralController>(dim, is_occupied, instances, file_path, 
                                                                   time_interval, 
                                                                   true); // enable opencv window
    executor2.add_node(central_controller);
    //executor.spin();


   
    std::thread t2([&]() { executor2.spin(); });

    // 250825 13：31
    // when do not use gazebo gui, every thing is ok,
    // but when use it, some agent node will not work (if gazebo node and other node are in the same executor)
    // draw gazebo gui
    rclcpp::executors::MultiThreadedExecutor executor3;
    auto gazebo_node = std::make_shared<GazeboGUI>(central_controller);  
    executor3.add_node(gazebo_node);
    std::thread t3([&]() { executor3.spin(); });
    t3.join();  

    t1.join();
    t2.join();
    

    rclcpp::shutdown();
    return 0;
}