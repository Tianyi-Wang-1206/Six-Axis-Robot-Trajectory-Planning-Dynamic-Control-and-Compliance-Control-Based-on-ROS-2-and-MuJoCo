#ifndef LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
#define LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lite6_interfaces/msg/controller_status.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace lite6_controllers
{

class Lite6CTCController : public controller_interface::ChainableControllerInterface
{
public:
  Lite6CTCController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;

  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;

  Eigen::VectorXd e_, de_, a_d_, tau_fric_, tau_cmd_;
  Eigen::VectorXd q_target_, dq_target_, ddq_target_;

  Eigen::VectorXd q_, dq_;

  // PD Control Matrices
  Eigen::MatrixXd Kp_, Kv_;

  // Integral acceleration (Ki * integral(e dt)); converted through M + armature.
  void apply_integral(double dt);
  Eigen::VectorXd integral_gain_, integral_acceleration_, integral_torque_;
  Eigen::VectorXd integral_torque_limit_;
  Eigen::MatrixXd integral_mass_;
  double integral_position_window_ = 0.017453292519943295;  // rad (1 degree)
  double integral_velocity_window_ = 0.02;  // rad/s, both reference and feedback
  bool integral_enabled_ = false;
  
  Eigen::VectorXd friction_v_, friction_c_, armature_;
  Eigen::VectorXd max_effort_;
  double tanh_slope_;

  enum class SystemState { NORMAL, ESTOP, SYS_ID };
  SystemState system_state_ = SystemState::NORMAL;
  
  // Kinematic Deceleration Tracker
  Eigen::VectorXd estop_q_, estop_dq_; 
  Eigen::VectorXd estop_decel_;
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
  realtime_tools::RealtimeBuffer<int32_t> cmd_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr shadow_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_shadow_pub_;

  // Observational CTC mode feed for the external Supervisor.
  // It does not own or change this controller's internal SystemState.
  rclcpp::Publisher<lite6_interfaces::msg::ControllerStatus>::SharedPtr status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lite6_interfaces::msg::ControllerStatus>> rt_status_pub_;
  int status_publish_rate_divider_ = 20;  // 1000 Hz / 20 = 50 Hz heartbeat
  int status_publish_counter_ = 0;
  SystemState last_reported_state_ = SystemState::NORMAL;
  bool status_publish_pending_ = true;

  // Downsampler to match the 200Hz JSB rate without flooding the network
  int publish_rate_divider_ = 5; 
  int publish_counter_ = 0;
};

}  // namespace lite6_controllers

#endif  // LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
