#ifndef LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
#define LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <array>
#include <cstdint>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lite6_interfaces/msg/controller_status.hpp"
#include "lite6_interfaces/msg/compliance_command.hpp"
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
  enum class TrajectoryResult {
    RUNNING, SUCCEEDED, CANCELED, DEACTIVATED, PATH_POSITION, PATH_VELOCITY,
    GOAL_TIMEOUT, SATURATION, INVALID_STATE, INVALID_COMMAND, COMMAND_WRITE, MODE_CHANGED,
    CONTROL_EXCEPTION
  };
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
    bool cancel_applied = false;
    int start_mode = 0;
    double settled_duration = 0.0;
    bool was_settled = false;
    std::array<double, 6> saturation_duration{};
    // Written before publishing a terminal result; read only after that result.
    int failure_joint = -1;
    double failure_value = 0.0;
    double failure_limit = 0.0;
  };
  void configure_trajectory_action();
  bool valid_trajectory_goal(const FollowTrajectory::Goal & goal) const;
  void monitor_trajectory();
  void check_trajectory_path();
  void finish_trajectory_cycle(double dt);
  void fail_trajectory(TrajectoryResult reason, int joint = -1,
    double value = 0.0, double limit = 0.0);
  controller_interface::return_type fail_controller(TrajectoryResult reason, int joint = -1);
  void request_execution_stop();
  // Borrowed from the RT buffer once per update; valid until the next RT read.
  TrajectoryExecution * current_execution_ = nullptr;
  std::vector<double> path_position_tolerance_, path_velocity_tolerance_;
  std::vector<double> goal_position_tolerance_, goal_velocity_tolerance_;
  std::vector<double> sysid_goal_position_tolerance_, sysid_goal_velocity_tolerance_;
  double goal_settle_duration_ = 0.3;
  double goal_time_tolerance_ = 5.0;
  double saturation_timeout_ = 1.0;
  std::array<bool, 6> effort_saturated_{};
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
  // Full articulated dynamics; command/reference/estop vectors stay six-axis.
  Eigen::VectorXd dynamics_q_, dynamics_dq_, dynamics_ddq_;

  Eigen::VectorXd e_, de_, a_d_, tau_fric_, tau_cmd_;
  Eigen::VectorXd q_target_, dq_target_, ddq_target_;

  Eigen::VectorXd q_, dq_;

  // PD Control Matrices
  Eigen::MatrixXd Kp_, Kv_;

  // Torque-domain integral: gains in Nm/(rad s), back-calculation in 1/s,
  // and both the retained state and its limits in Nm.
  Eigen::VectorXd ki_, antiwindup_gain_, integral_max_effort_, integral_effort_;
  Eigen::VectorXd friction_v_, friction_c_, armature_;
  Eigen::VectorXd friction_error_gain_;
  Eigen::VectorXd max_effort_;
  double tanh_slope_;

  enum class SystemState { NORMAL, ESTOP, SYS_ID, JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE, TEACH, COMPLIANCE_STOPPING };
  bool compliance_active() const;
  void configure_compliance();
  void process_compliance_command(int32_t command, const lite6_interfaces::msg::ComplianceSettings& settings);
  void compute_compliance(double dt);
  lite6_interfaces::msg::ComplianceSettings compliance_settings_, compliance_filtered_;
  Eigen::Matrix<double,6,1> compliance_q_=Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,1> compliance_last_tau_=Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,Eigen::Dynamic> compliance_jacobian_;
  pinocchio::SE3 compliance_pose_;
  pinocchio::FrameIndex compliance_frame_=0;
  uint64_t compliance_sequence_=0;
  double compliance_saturation_time_=0.0;
  std::atomic<bool> compliance_blocks_trajectory_{false};
  std::atomic<bool> emergency_pending_{false};
  rclcpp::Subscription<lite6_interfaces::msg::ComplianceCommand>::SharedPtr compliance_sub_;
  SystemState system_state_ = SystemState::NORMAL;
  
  // Kinematic Deceleration Tracker
  Eigen::VectorXd estop_q_, estop_dq_; 
  Eigen::VectorXd estop_decel_;
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
  struct ModeCommand {
    int32_t value=0; uint64_t sequence=0;
    lite6_interfaces::msg::ComplianceSettings settings;
    ModeCommand()=default;
    ModeCommand(int32_t v,uint64_t s):value(v),sequence(s) {}
  };
  realtime_tools::RealtimeBuffer<ModeCommand> cmd_buffer_;
  std::atomic<uint64_t> command_sequence_{0};
  uint64_t current_command_sequence_ = 0;
  uint64_t execution_stop_sequence_ = 0;
  bool execution_stop_latched_ = false;

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

};

}  // namespace lite6_controllers

#endif  // LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
