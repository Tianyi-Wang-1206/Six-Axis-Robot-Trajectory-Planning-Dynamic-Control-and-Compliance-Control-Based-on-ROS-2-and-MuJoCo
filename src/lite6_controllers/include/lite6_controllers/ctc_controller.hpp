#ifndef LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
#define LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lite6_interfaces/msg/controller_status.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace lite6_controllers
{

class Lite6CTCController : public controller_interface::ControllerInterface
{
public:
  Lite6CTCController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void update_trajectory_reference(const rclcpp::Time & time, const rclcpp::Duration & period);
  controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period);

private:
  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  using TrajectoryGoal = rclcpp_action::ServerGoalHandle<FollowTrajectory>;
  enum class TrajectoryResult { RUNNING, SUCCEEDED, CANCELED, DEACTIVATED };
  struct TrajectoryExecution {
    explicit TrajectoryExecution(std::shared_ptr<TrajectoryGoal> handle) : goal(std::move(handle)) {}
    const std::shared_ptr<TrajectoryGoal> goal;
    std::atomic<bool> cancel{false};
    std::atomic<TrajectoryResult> result{TrajectoryResult::RUNNING};
    std::atomic<size_t> sample{0};
    // Only the control thread accesses these playback fields.
    double elapsed = 0.0;
    int64_t start_time_ns = 0;
    bool started = false;
    bool endpoint_applied = false;
  };
  void configure_trajectory_action();
  bool valid_trajectory_goal(const FollowTrajectory::Goal & goal) const;
  void monitor_trajectory();
  std::vector<double> reference_interfaces_;
  std::atomic<bool> trajectory_enabled_{false};
  std::mutex trajectory_mutex_;  // Non-real-time callbacks only.
  bool trajectory_reserved_ = false;
  std::shared_ptr<TrajectoryExecution> active_trajectory_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajectoryExecution>> trajectory_buffer_;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr trajectory_server_;
  rclcpp::TimerBase::SharedPtr trajectory_monitor_;

  std::vector<std::string> joint_names_;

  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;

  Eigen::VectorXd e_, de_, a_d_, tau_fric_, tau_cmd_;
  Eigen::VectorXd q_target_, dq_target_, ddq_target_;

  Eigen::VectorXd q_, dq_;

  // PD Control Matrices
  Eigen::MatrixXd Kp_, Kv_;

  // Torque-domain integral: gains in Nm/(rad s), back-calculation in 1/s,
  // and both the retained state and its limits in Nm.
  Eigen::VectorXd ki_, antiwindup_gain_, integral_max_effort_, integral_effort_;
  Eigen::VectorXd friction_v_, friction_c_, friction_s_, friction_vs_, armature_;
  Eigen::VectorXd friction_error_gain_;
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

  // Downsample shadow visualization to 200Hz without flooding the network.
  int publish_rate_divider_ = 5; 
  int publish_counter_ = 0;
};

}  // namespace lite6_controllers

#endif  // LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
