#ifndef FAKE_AGENTS
#define FAKE_AGENTS

#include <cstdio>
//#include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>

#include <fstream>
#include <sstream>
#include <string>

#include "common_interfaces.h"
#include  "tinyxml.h"

void createCylinderSDFFile(const std::string& model_name, const std::string& file_path, double radius, double height,
                           cv::Vec3b color = cv::Vec3b::all(0)) {
    // SDF 模板，定义一个带圆柱体的模型
    std::string sdf_content = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <gravity>0</gravity>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius2}</radius>
            <length>{height2}</length>
          </cylinder>
        </geometry>
        <material> <!-- Start material -->
          <ambient>{R} {G} {B} 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material> <!-- End material -->
      </visual>
    </link>
  </model>
</sdf>
)";

    // 替换模型名称、半径和高度
    size_t pos = sdf_content.find("{model_name}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{model_name}").length(), model_name);
    }
    pos = sdf_content.find("{radius1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{radius1}").length(), std::to_string(radius));
    }
    pos = sdf_content.find("{height1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height1}").length(), std::to_string(height));
    }
    pos = sdf_content.find("{radius2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{radius2}").length(), std::to_string(radius));
    }
    pos = sdf_content.find("{height2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height2}").length(), std::to_string(height));
    }

    // 设置颜色
    //在 OpenCV 中，颜色为 BGR，而不是 RGB
    pos = sdf_content.find("{R}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{R}").length(), std::to_string(color[0]/255.));
    }
    pos = sdf_content.find("{G}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{G}").length(), std::to_string(color[1]/255.));
    }
    pos = sdf_content.find("{B}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{B}").length(), std::to_string(color[2]/255.));
    }
    // std::cout << " sdf_content = " << sdf_content << std::endl;

    // 将SDF内容写入文件
    std::ofstream sdf_file(file_path);
    if (sdf_file.is_open()) {
        sdf_file << sdf_content;
        sdf_file.close();
        // std::cout << "SDF file created with cylinder at: " << file_path << std::endl;
    } else {
        std::cerr << "Failed to create cylinder SDF file at: " << file_path << std::endl;
    }
}

void createBlockSDFFile(const std::string& model_name, const std::string& file_path, 
                       const freeNav::Pointf<2>& pt_min, const freeNav::Pointf<2>& pt_max,
                       double height, cv::Vec3b color = cv::Vec3b::all(0)) {

    double length = pt_max[0] - pt_min[0];
    double width  = pt_max[1] - pt_min[1];
    double x      = (pt_max[0] + pt_min[0])/2;
    // SDF 模板，定义一个带圆柱体的模型
    std::string sdf_content = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <link name="base_link">
      <pose>{x} 0 0 0 0 0</pose>
      <static>true</static>
      <gravity>0</gravity>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length2} {width2} {height}</size>
          </box>
        </geometry>
        <material> <!-- Start material -->
          <ambient>{R} {G} {B} 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 1</emissive>
        </material> <!-- End material -->
      </visual>
    </link>
  </model>
</sdf>
)";

    // 替换模型名称、半径和高度
    size_t pos = sdf_content.find("{model_name}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{model_name}").length(), model_name);
    }
    pos = sdf_content.find("{x}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{x}").length(), std::to_string(x));
    }

    pos = sdf_content.find("{length1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{length1}").length(), std::to_string(length));
    }
    pos = sdf_content.find("{width1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{width1}").length(), std::to_string(width));
    }
    pos = sdf_content.find("{length2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{length2}").length(), std::to_string(length));
    }
    pos = sdf_content.find("{width2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{width2}").length(), std::to_string(width));
    }
    pos = sdf_content.find("{height}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height}").length(), std::to_string(height));
    }
    pos = sdf_content.find("{height}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height}").length(), std::to_string(height));
    }



    // 设置颜色
    //在 OpenCV 中，颜色为 BGR，而不是 RGB
    pos = sdf_content.find("{R}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{R}").length(), std::to_string(color[0]/255.));
    }
    pos = sdf_content.find("{G}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{G}").length(), std::to_string(color[1]/255.));
    }
    pos = sdf_content.find("{B}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{B}").length(), std::to_string(color[2]/255.));
    }

    // 将SDF内容写入文件
    std::ofstream sdf_file(file_path);
    if (sdf_file.is_open()) {
        sdf_file << sdf_content;
        sdf_file.close();
        // std::cout << "SDF file created with box at: " << file_path << std::endl;
    } else {
        std::cerr << "Failed to create box SDF file at: " << file_path << std::endl;
    }
}

std::string loadSDFFile(const std::string& file_path) {
    std::ifstream sdf_file(file_path);
    std::stringstream sdf_buffer;
    sdf_buffer << sdf_file.rdbuf();
    return sdf_buffer.str();
}

std::string createCircleAgent(const std::string& model_name, double radius, double height, cv::Vec3b color = cv::Vec3b::all(0)) {
    std::string file_path = "/home/yaozhuo/Desktop/fake_large_agents/" + model_name + ".sdf";
    createCylinderSDFFile(model_name, file_path, radius, height, color);
    return file_path;
}

std::string createBlockAgent(const std::string& model_name,
                             const freeNav::Pointf<2>& pt_min,
                             const freeNav::Pointf<2>& pt_max,
                             double height,
                             cv::Vec3b color = cv::Vec3b::all(0)) {
    std::string file_path = "/home/yaozhuo/Desktop/fake_large_agents/" + model_name + ".sdf";
    createBlockSDFFile(model_name, file_path, pt_min, pt_max, height, color);
    return file_path;
}

typedef rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr SpawnClientPtr;

typedef rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr SetPoseClientPtr;

bool spawnAgentGazebo(const std::string& file_path, const std::string model_name, 
                      const geometry_msgs::msg::Pose& initial_pose, 
                      const SpawnClientPtr& client, const rclcpp::Node::SharedPtr& node
                      ) {


    // 读取SDF文件内容
    //std::string model_xml = loadSDFFile(file_path);
    // 加载 XML 文件
    TiXmlDocument doc(file_path);
    if (!doc.LoadFile()) {
        std::cerr << "unable to load sdf file " << file_path <<  ", error code " << doc.ErrorDesc() << std::endl;
        return 1;
    }
    std::cout << " file_path = " << file_path << std::endl;
    // std::cout << " model_xml = " << model_xml << std::endl;

    // set name in xml file to model_name
    // 获取根元素
    TiXmlElement* root = doc.FirstChildElement("sdf");
    if (!root) {
        std::cerr << "cannot find element sdf" << std::endl;
        return 1;
    }
    TiXmlElement* model_element = root->FirstChildElement("model");
    if (!model_element) {
        std::cerr << "cannot find element model" << std::endl;
        return 1;
    } else {
        //std::cerr << "find element model" << std::endl;
        std::cout << model_element->GetText() << std::endl;
    }
    model_element->SetAttribute("name", model_name.c_str());

    TiXmlPrinter printer;
    doc.Accept(&printer);
    std::string xmlContent = printer.Str();
    std::cout << "XML content:\n" << xmlContent << std::endl;

    // 创建SpawnEntity请求
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;              // 设置模型名称
    request->xml = xmlContent;                // SDF文件内容
    request->robot_namespace = "";           // 机器人命名空间，可留空
    request->initial_pose = initial_pose;    // 设置模型初始位置
    request->reference_frame = "world";      // 设置参考坐标系

    // 发送请求并等待响应
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Successfully spawned model: %s", model_name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to spawn model: %s", model_name.c_str());
        return false;
    }
    return true;
}


void setModelPose(std::string model_name, double x, double y, double z, double theta, 
                  const SetPoseClientPtr& client,
                  const rclcpp::Node::SharedPtr& node) {

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, theta); // Create this quaternion from roll/pitch/yaw (in radians)

    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();

    // 创建请求
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name.c_str();
    request->state.pose = pose;

    // 异步发送请求
    auto result_future = client->async_send_request(request);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(node->get_logger(), "模型 %s 位姿已更新", model_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "无法更新模型位姿");
    }
}

#endif