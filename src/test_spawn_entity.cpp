 #include "lamapf_and_gazebo/spawn_entity.hpp"


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
  std::string sdf_filename = "/home/yaozhuo/code/ros2_ws/src/ros_gz/ros_gz_sim_demos/models/cardboard_box/model.sdf";
  
  // Create spawner and call service
  auto spawner = std::make_shared<EntitySpawner>();
  bool result1 =
    spawner->spawn_entity(model_name, sdf_filename, pose);

  model_name = "my_vehicle_yz_2";
  sdf_filename = "/home/yaozhuo/code/ros2_ws/src/ros_gz/ros_gz_sim_demos/models/vehicle/model.sdf";
  
  pose.position.x = 2.0;
  pose.position.y = 2.0;
  pose.position.z = 0.0;

  bool result2 =
    spawner->spawn_entity(model_name, sdf_filename, pose);

  rclcpp::shutdown();
  return result1 ? 0 : 1;
}
