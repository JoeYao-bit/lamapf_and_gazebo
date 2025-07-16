#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include <chrono>
#include <fstream>
#include <streambuf>

using namespace std::chrono_literals;

std::string read_file(const std::string &file_path) {
  std::ifstream t(file_path);
  if (!t) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }
  return std::string((std::istreambuf_iterator<char>(t)),
                     std::istreambuf_iterator<char>());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spawn_entity_client");

  auto client = node->create_client<ros_gz_interfaces::srv::SpawnEntity>("/spawn_entity");

  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Waiting for spawn_entity service...");
  }

  auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();

  request->entity_factory;

  // 加载模型文件，例如URDF或SDF
  try {
    request->entity_factory.sdf = read_file("/home/yaozhuo/.gazebo_model_yz/models/ambulance/model.sdf");
  } catch (std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Error reading model file: %s", e.what());
    return 1;
  }

  request->entity_factory.name = "my_robot";
  // request->robot_namespace = "robot_ns";
  request->entity_factory.pose.position.x = 0.0;
  request->entity_factory.pose.position.y = 0.0;
  request->entity_factory.pose.position.z = 0.5;
  request->entity_factory.pose.orientation.w = 1.0;

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Entity spawned successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call spawn_entity.");
  }

  rclcpp::shutdown();
  return 0;
}

