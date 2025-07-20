// Copyright 2025 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file set_entity_pose.hpp
/// \brief Defines utilities and a ROS 2 node for setting an entity's pose in a Gazebo simulation.

#ifndef ROS_GZ_SIM__SET_ENTITY_POSE_HPP_
#define ROS_GZ_SIM__SET_ENTITY_POSE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

/// \brief A ROS 2 node for setting entity poses in a Gazebo simulation.
class EntityPoseSetter : public rclcpp::Node {
public:
  /// \brief Default constructor. Initializes the node and pose setter client.
  EntityPoseSetter() : Node("entity_pose_setter")
  {
    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/default/set_pose");
  }

  /// \brief Constructor that accepts an external pose service client.
  /// \param[in] client Shared pointer to the set entity pose service client.
  explicit EntityPoseSetter(
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client)
    : Node("entity_pose_setter"), client_(client) {}

  /// \brief Sets the pose of an entity in the simulation.
  /// \param[in] entity_name Name of the entity.
  /// \param[in] entity_id ID of the entity.
  /// \param[in] entity_type Type of the entity.
  /// \param[in] x X position coordinate.
  /// \param[in] y Y position coordinate.
  /// \param[in] z Z position coordinate.
  /// \param[in] qx First orientation parameter (quaternion X component or roll angle).
  /// \param[in] qy Second orientation parameter (quaternion Y component or pitch angle).
  /// \param[in] qz Third orientation parameter (quaternion Z component or yaw angle).
  /// \param[in] qw Fourth orientation parameter (quaternion W component, ignored for ]
  ///            Euler angles).
  /// \param[in] use_quaternion If true, orientation parameters are interpreted as quaternion
  ///            components. If false, they are interpreted as Euler angles (roll, pitch, yaw).
  /// \return True if the entity pose was set successfully, false otherwise.
  bool set_entity_pose(
    const std::string & entity_name, int entity_id,
    int entity_type, double x, double y, double z, double qx,
    double qy, double qz, double qw,
    bool use_quaternion = true) {
    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                      "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // Create the request
    auto request =
      std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    auto entity = ros_gz_interfaces::msg::Entity();

    // Set entity identification (name or ID)
    if (entity_id > 0) {
      entity.id = entity_id;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with ID: %d",
                  entity_id);
    } else if (!entity_name.empty()) {
      entity.name = entity_name;
      RCLCPP_INFO(this->get_logger(), "Setting pose for entity with name: %s",
                  entity_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                  "Either entity name or ID must be provided");
      return false;
    }

    entity.type = entity_type;
    request->entity = entity;

    // Set position
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Set orientation (quaternion)
    if (use_quaternion) {
      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
    } else {
      // In this case, qx=roll, qy=pitch, qz=yaw (in radians)
      // Use tf2 to convert Euler angles to quaternion
      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(qx, qy, qz);

      // Convert to geometry_msgs quaternion
      pose.orientation.x = tf2_quat.x();
      pose.orientation.y = tf2_quat.y();
      pose.orientation.z = tf2_quat.z();
      pose.orientation.w = tf2_quat.w();

      // Update qw for logging
      qw = pose.orientation.w;
    }

    request->pose = pose;

    RCLCPP_INFO(this->get_logger(), "Position: x=%f, y=%f, z=%f",
                pose.position.x, pose.position.y, pose.position.z);

    RCLCPP_INFO(this->get_logger(), "Orientation: x=%f, y=%f, z=%f, w=%f",
                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w);

    // Send the request
    auto future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                          future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s",
                  response->success ? "true" : "false");

      if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set entity pose");
        return false;
      }
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

protected:
  /// \brief Client used to call the set entity pose service.
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
};

typedef std::shared_ptr<EntityPoseSetter> EntityPoseSetterPtr;


#endif  // ROS_GZ_SIM__SET_ENTITY_POSE_HPP_
