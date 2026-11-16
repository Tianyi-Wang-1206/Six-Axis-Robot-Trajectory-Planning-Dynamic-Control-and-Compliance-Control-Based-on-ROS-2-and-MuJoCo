#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

/**
 * Lightweight latched publisher for /robot_description required by ros2_control
 * controller_manager in ROS 2 Jazzy when robot_state_publisher and TF are omitted.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_description_publisher");
  const std::string urdf = node->declare_parameter<std::string>("robot_description", "");
  if (urdf.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'robot_description' is empty; cannot initialize controller_manager");
    rclcpp::shutdown();
    return 1;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto pub = node->create_publisher<std_msgs::msg::String>("robot_description", qos);
  std_msgs::msg::String msg;
  msg.data = urdf;
  pub->publish(msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
