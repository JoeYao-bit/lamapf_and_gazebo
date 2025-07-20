 #include "lamapf_and_gazebo/spawn_entity.hpp"
 #include "lamapf_and_gazebo/delete_entity.hpp"
 #include "lamapf_and_gazebo/set_entity_pose.hpp"
 #include "lamapf_and_gazebo/common_interfaces.h"

 /// \brief Main entry point for the entity spawner node.
 /// \details Parses arguments, constructs a pose, and calls the spawn service.
 /// \param[in] argc Number of command-line arguments.
 /// \param[in] argv List of command-line arguments.
 /// \return Exit status code: 0 for success, 1 for failure.
int main(int argc, char **argv)
{
   // Initialize ROS
  rclcpp::init(argc, argv);


  // Set up geometry_msgs::msg::Pose with default values
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  // Use tf2 for Euler to quaternion conversion
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, 0);

    // Copy to pose orientation
  pose.orientation.x = tf2_quat.x();
  pose.orientation.y = tf2_quat.y();
  pose.orientation.z = tf2_quat.z();
  pose.orientation.w = tf2_quat.w();

  std::string model_name = "my_vehicle_yz";
  std::string sdf_filename = ROBOT_SDFS[1];
  //"/home/yaozhuo/code/ros2_ws/src/ros_gz/ros_gz_sim_demos/models/cardboard_box/model.sdf";
  
  // Create spawner and call service
  auto spawner = std::make_shared<EntitySpawner>();
  bool result =
    spawner->spawn_entity(model_name, sdf_filename, pose);

  model_name = "my_vehicle_yz_2";
  sdf_filename = ROBOT_SDFS[1];
  //"/home/yaozhuo/code/ros2_ws/src/ros_gz/ros_gz_sim_demos/models/vehicle/model.sdf";
  
  pose.position.x = 2.0;
  pose.position.y = 2.0;
  pose.position.z = 0.0;

  result = spawner->spawn_entity(model_name, sdf_filename, pose);


  model_name = "my_vehicle_yz_2";  

  // Create deleter and call service
  auto deleter = std::make_shared<EntityDeleter>();
  result = deleter->delete_entity(model_name, 0, 6);  

    // Set defaults for orientation
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  bool use_quaternion = true;

  // // Apply orientation if provided
  // if (!quaternion.empty()) {
  //   qx = quaternion[0];
  //   qy = quaternion[1];
  //   qz = quaternion[2];
  //   qw = quaternion[3];
  //   use_quaternion = true;
  // } else if (!euler.empty()) {
  //   qx = euler[0];    // roll
  //   qy = euler[1];    // pitch
  //   qz = euler[2];    // yaw
  //   use_quaternion = false;
  // }
  model_name = "my_vehicle_yz";  

  // Create pose setter and call service
  auto pose_setter = std::make_shared<EntityPoseSetter>();
  result = pose_setter->set_entity_pose(model_name, 0, 6, 0, 0, 2,
                                  0, 0, 0, 1.0, false);

  rclcpp::shutdown();
  return result ? 0 : 1;
}
