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

/// \file delete_entity.hpp
/// \brief Defines utilities and a ROS 2 node for deleting entities from a Gazebo simulation.

#ifndef ROS_GZ_SIM__DELETE_ENTITY_HPP_
#define ROS_GZ_SIM__DELETE_ENTITY_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

/// \brief A ROS 2 node for deleting entities from a Gazebo simulation.
class EntityDeleter : public rclcpp::Node {
public:
  /// \brief Default constructor. Initializes the node and delete entity client.
  EntityDeleter(): Node("entity_deleter")
  {
    client_ = create_client<ros_gz_interfaces::srv::DeleteEntity>(
      "/world/default/remove");
  }

  /// \brief Constructor that accepts an external delete service client.
  /// \param[in] client Shared pointer to the delete entity service client.
  explicit EntityDeleter(
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client)
    : Node("entity_deleter"), client_(client) {}

  /// \brief Deletes an entity from the simulation.
  /// \param[in] entity_name Name of the entity.
  /// \param[in] entity_id ID of the entity.
  /// \param[in] entity_type Type of the entity.
  /// \return True if the entity was deleted successfully, false otherwise.
  bool delete_entity(
    const std::string & entity_name, int entity_id,
    int entity_type) {
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
      std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    auto entity = ros_gz_interfaces::msg::Entity();

    // Set entity identification (name or ID)
    if (entity_id > 0) {
      entity.id = entity_id;
      RCLCPP_INFO(this->get_logger(), "Deleting entity with ID: %d", entity_id);
    } else if (!entity_name.empty()) {
      entity.name = entity_name;
      RCLCPP_INFO(this->get_logger(), "Deleting entity with name: %s",
                  entity_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                    "Either entity name or ID must be provided");
      return false;
    }

    entity.type = entity_type;
    request->entity = entity;

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
        RCLCPP_ERROR(this->get_logger(), "Failed to delete entity");
        return false;
      }
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return false;
    }
  }

protected:
  /// \brief Client used to call the delete entity service.
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr client_;
};

typedef std::shared_ptr<EntityDeleter> EntityDeleterPtr;


#endif  // ROS_GZ_SIM__DELETE_ENTITY_HPP_
