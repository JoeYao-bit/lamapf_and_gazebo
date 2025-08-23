#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "lamapf_and_gazebo/common_interfaces.h"

#include "lamapf_and_gazebo/controller.h"
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
    rclcpp::executors::MultiThreadedExecutor executor;
    // 我发现executor.add_node放在循环中，比如for循化，那么time_中的内容不会运行，但如果一个个手动添加，则会运行，这是为什么
    // executor.add_node() 只保存了节点的 裸指针（在内部包装成 weak_ptr）
    // 如果你在循环中创建的节点没有被其他变量持有，它会在循环迭代结束时析构
    // 析构后，Timer 对象也被销毁，回调自然不会触发
    // 外部保存，即可避免这一缺陷。

    std::vector<std::shared_ptr<LocalController> > nodes;
    for(int i=0; i<agents.size(); i++) {
        auto agent_node = std::make_shared<LocalController>(agents[i], line_ctls[i], rot_ctls[i], 1);
        executor.add_node(agent_node);
        nodes.push_back(agent_node); 
    }
    // start central controller
    auto central_controller = std::make_shared<CenteralController>(dim, is_occupied, instances);
    executor.add_node(central_controller);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}