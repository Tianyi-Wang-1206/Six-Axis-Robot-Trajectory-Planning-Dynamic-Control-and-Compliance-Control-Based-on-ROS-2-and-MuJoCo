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
#include "lite6_interfaces/msg/controller_status.hpp"
#include "lite6_interfaces/msg/compliance_command.hpp"
#include "lite6_interfaces/action/follow_impedance_trajectory.hpp"
#include "lite6_interfaces/shm_registers.hpp"
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
  void latch_estop_reason(uint8_t reason, int joint = -1, double value = 0.0, double limit = 0.0);
  void clear_estop_reason();
  void publish_shadow_shm(const Eigen::VectorXd * target, bool valid);
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

  enum class SystemState {
    NORMAL,
    ESTOP,
    SYS_ID,
    JOINT_IMPEDANCE,
    CARTESIAN_IMPEDANCE,
    TEACH,
    COMPLIANCE_STOPPING,
    CARTESIAN_IMPEDANCE_TRAJECTORY
  };
  bool compliance_active() const;
  bool impedance_trajectory_active() const;
  bool configure_compliance();
  void process_compliance_command(double dt, int32_t command, const lite6_interfaces::msg::ComplianceSettings& settings);
  void compute_compliance(double dt);

  using FollowImpedance = lite6_interfaces::action::FollowImpedanceTrajectory;
  using ImpedanceGoal = rclcpp_action::ServerGoalHandle<FollowImpedance>;
  enum class ImpedanceTrajectoryResult {
    RUNNING,
    SUCCEEDED,
    CANCELED,
    DEACTIVATED,
    DEVIATION_LIMIT,
    VELOCITY_LIMIT,
    POSITION_LIMIT,
    SATURATION,
    EXTERNAL_TORQUE_LIMIT,
    GOAL_TIMEOUT,
    MODE_CHANGED,
    INVALID_STATE,
    INVALID_COMMAND,
    COMMAND_WRITE,
    CONTROL_EXCEPTION
  };
  struct ImpedanceTrajectoryExecution {
    explicit ImpedanceTrajectoryExecution(std::shared_ptr<ImpedanceGoal> handle) : goal(std::move(handle)) {}
    const std::shared_ptr<ImpedanceGoal> goal;
    std::atomic<bool> cancel{false};
    std::atomic<ImpedanceTrajectoryResult> result{ImpedanceTrajectoryResult::RUNNING};
    std::atomic<size_t> sample{0};
    std::array<std::atomic<double>, 3> pos_error{{0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 3> rot_error{{0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> external_torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> external_wrench{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::atomic<bool> contact_detected{false};
    // Control thread only fields.
    double elapsed = 0.0;
    int64_t start_time_ns = 0;
    bool started = false;
    bool endpoint_applied = false;
    bool cancel_applied = false;
    double settled_duration = 0.0;
    bool was_settled = false;
    double saturation_duration = 0.0;
    double external_torque_duration = 0.0;
    int failure_joint = -1;
    double failure_value = 0.0;
    double failure_limit = 0.0;
    std::string failure_detail;
  };
  bool configure_impedance_trajectory();
  bool valid_impedance_trajectory_goal(const FollowImpedance::Goal & goal) const;
  void update_impedance_trajectory_reference(const rclcpp::Time & time, const rclcpp::Duration & period);
  void compute_impedance_trajectory(double dt);
  void check_impedance_trajectory_path();
  void finish_impedance_trajectory_cycle(double dt);
  void fail_impedance_trajectory(ImpedanceTrajectoryResult reason, int joint = -1,
    double value = 0.0, double limit = 0.0, const std::string & detail = "");
  void monitor_impedance_trajectory();

  ImpedanceTrajectoryExecution * current_impedance_execution_ = nullptr;
  bool impedance_trajectory_reserved_ = false;
  std::shared_ptr<ImpedanceTrajectoryExecution> active_impedance_trajectory_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ImpedanceTrajectoryExecution>> impedance_trajectory_buffer_;
  rclcpp_action::Server<FollowImpedance>::SharedPtr impedance_trajectory_server_;

  pinocchio::Data pin_data_ref_;
  Eigen::VectorXd dynamics_q_ref_;
  Eigen::VectorXd dynamics_dq_ref_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imp_ref_jacobian_;
  Eigen::Matrix<double, 6, 1> q_imp_ref_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> dq_imp_ref_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> ddq_imp_ref_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d imp_kt_target_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d imp_dt_target_ = Eigen::Vector3d::Constant(25.0);
  Eigen::Vector3d imp_kr_target_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d imp_dr_target_ = Eigen::Vector3d::Constant(1.5);
  double imp_null_damping_target_ = 0.15;
  Eigen::Vector3d imp_kt_filtered_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d imp_dt_filtered_ = Eigen::Vector3d::Constant(25.0);
  Eigen::Vector3d imp_kr_filtered_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d imp_dr_filtered_ = Eigen::Vector3d::Constant(1.5);
  double imp_null_damping_filtered_ = 0.15;
  Eigen::Vector3d imp_current_pos_error_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d imp_current_rot_error_ = Eigen::Vector3d::Zero();
  std::array<bool, 6> imp_effort_saturated_{};

  double imp_max_trans_deviation_ = 0.12;
  double imp_max_rot_deviation_ = 0.60;
  double imp_max_joint_deviation_ = 0.80;
  double imp_max_joint_velocity_ = 3.0;
  double imp_hard_limit_tolerance_ = 0.01;
  double imp_saturation_timeout_ = 1.0;
  double imp_goal_trans_tolerance_ = 0.05;
  double imp_goal_rot_tolerance_ = 0.25;
  double imp_goal_velocity_tolerance_ = 0.01;
  double imp_goal_settle_duration_ = 0.3;
  double imp_goal_time_tolerance_ = 5.0;
  double imp_entry_max_speed_ = 0.02;
  double imp_entry_position_tolerance_ = 0.15;
  double compliance_max_joint_velocity_ = 1.5;
  double compliance_hard_limit_tolerance_ = 0.01;
  double compliance_saturation_timeout_ = 1.0;
  double compliance_entry_max_speed_ = 0.01;
  double compliance_filter_time_constant_ = 0.15;
  double compliance_cartesian_null_damping_ = 0.15;
  double compliance_soft_limit_margin_ = 0.15;
  double compliance_soft_limit_stiffness_ = 80.0;
  double compliance_soft_limit_damping_ = 5.0;
  double compliance_torque_slew_rate_ = 100.0;
  double compliance_fv_scale_ = 0.85;
  double compliance_fc_scale_ = 0.0;
  double compliance_fric_trans_speed_ = 0.06;
  double compliance_fric_quiet_speed_ = 0.08;
  std::array<double, 6> compliance_max_external_torque_{35.0, 35.0, 24.0, 20.0, 20.0, 12.0};
  double compliance_external_torque_timeout_ = 0.25;
  double compliance_external_torque_time_ = 0.0;
  double imp_fv_scale_ = 0.90;
  double imp_fc_scale_ = 0.80;
  double imp_fric_trans_speed_ = 0.04;
  double imp_fric_quiet_speed_ = 0.01;
  std::array<double, 6> imp_max_external_torque_{35.0, 35.0, 24.0, 20.0, 20.0, 12.0};
  double imp_external_torque_timeout_ = 0.25;

  // Sensorless Generalized Momentum Observer (GMO) state & configuration.
  bool configure_momentum_observer();
  void reset_momentum_observer();
  void update_momentum_observer(double dt);
  void add_passive_friction_ff(
    double fv_scale, double fc_scale, double v_trans, double quiet_speed,
    const Eigen::Ref<const Eigen::VectorXd> & dq_coulomb,
    Eigen::VectorXd & tau_out) const;
  bool check_external_torque_limit(
    double dt, const std::array<double, 6> & limits, double timeout,
    double & accum_duration, int & out_joint, double & out_val, double & out_limit) const;

  bool gmo_enabled_ = true;
  bool gmo_initialized_ = false;
  std::array<double, 6> gmo_gains_{25.0, 25.0, 25.0, 30.0, 30.0, 30.0};
  double gmo_jacobian_damping_ = 0.04;
  double gmo_fric_trans_speed_ = 0.03;
  double gmo_contact_force_threshold_ = 2.5;
  pinocchio::Data pin_data_gmo_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> gmo_jacobian_;
  Eigen::Matrix<double, 6, 1> gmo_p_integral_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> gmo_residual_tau_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> gmo_external_wrench_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> tau_applied_prev_ = Eigen::Matrix<double, 6, 1>::Zero();
  bool gmo_contact_detected_ = false;
  std::array<double, 5> teach_damping_levels_{4.0, 3.25, 2.5, 2.25, 2.0};
  std::array<double, 5> teach_assistance_levels_{0.35, 0.475, 0.60, 0.625, 0.65};
  std::array<double, 5> teach_friction_transition_levels_{0.06, 0.06, 0.06, 0.06, 0.06};
  std::array<double, 5> teach_quiet_speed_levels_{0.06, 0.06, 0.06, 0.06, 0.06};
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

  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> state_shm_;
  uint64_t shadow_epoch_ = 0;

  // Observational CTC mode feed for the external Supervisor.
  // It does not own or change this controller's internal SystemState.
  rclcpp::Publisher<lite6_interfaces::msg::ControllerStatus>::SharedPtr status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<lite6_interfaces::msg::ControllerStatus>> rt_status_pub_;
  int status_publish_rate_divider_ = 20;  // 1000 Hz / 20 = 50 Hz heartbeat
  int status_publish_counter_ = 0;
  SystemState last_reported_state_ = SystemState::NORMAL;
  bool status_publish_pending_ = true;

  // RT-only latched diagnostics for the current ESTOP episode.
  uint8_t estop_reason_ = lite6_interfaces::msg::ControllerStatus::ESTOP_NONE;
  int8_t estop_joint_ = -1;
  double estop_value_ = 0.0;
  double estop_limit_ = 0.0;
  bool compliance_saturation_estop_pending_ = false;
  int8_t pending_estop_joint_ = -1;
  double pending_estop_value_ = 0.0;
  double pending_estop_limit_ = 0.0;

};

}  // namespace lite6_controllers

#endif  // LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
