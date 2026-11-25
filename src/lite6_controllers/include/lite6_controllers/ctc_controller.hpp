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
#include "realtime_tools/realtime_buffer.hpp"
#include "lite6_interfaces/msg/controller_status.hpp"
#include "lite6_interfaces/srv/controller_command.hpp"
#include "lite6_interfaces/srv/splice_joint_trajectory.hpp"
#include "lite6_interfaces/action/follow_impedance_trajectory.hpp"
#include "lite6_interfaces/motion_contract.hpp"
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
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Vec8d = Eigen::Matrix<double, 8, 1>;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Mat6x8d = Eigen::Matrix<double, 6, 8>;

  using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
  using TrajectoryGoal = rclcpp_action::ServerGoalHandle<FollowTrajectory>;
  using SpliceTrajectory = lite6_interfaces::srv::SpliceJointTrajectory;
  enum class TrajectoryResult {
    RUNNING, SUCCEEDED, CANCELED, DEACTIVATED, PATH_POSITION, PATH_VELOCITY,
    GOAL_TIMEOUT, SATURATION, INVALID_STATE, INVALID_COMMAND, COMMAND_WRITE, MODE_CHANGED,
    CONTROL_EXCEPTION, ENTRY_DEVIATION
  };
  enum class SpliceState : uint8_t {
    PENDING = 0,
    ADOPTED = 1,
    REJECTED_TIMING = 2,
    REJECTED_PARENT = 3,
    REJECTED_STATE = 4,
    WITHDRAWN = 5
  };
  struct SpliceExecution {
    trajectory_msgs::msg::JointTrajectory trajectory;
    double switch_time = 0.0;
    double t0_sec = 0.0;
    bool uniform_1khz = true;
    uint64_t controller_epoch = 0;
    std::atomic<uint8_t> state{static_cast<uint8_t>(SpliceState::PENDING)};
  };
  struct TrajectoryExecution {
    explicit TrajectoryExecution(std::shared_ptr<TrajectoryGoal> handle) : goal(std::move(handle)) {}
    const std::shared_ptr<TrajectoryGoal> goal;
    std::atomic<bool> cancel{false};
    std::atomic<TrajectoryResult> result{TrajectoryResult::RUNNING};
    std::atomic<size_t> sample{0};
    // Actual first-cycle time [ns] and elapsed time [s], published by RT.
    std::atomic<int64_t> start_time_pub_ns{0};
    std::atomic<double> elapsed_pub{0.0};
    // Populated during non-RT validation before RT handoff.
    bool uniform_1khz = true;
    double t0_sec = 0.0;
    // Path-preserving splice submitted via /ctc_controller/splice_joint_trajectory.
    // splice_owner holds the active pending or adopted splice under trajectory_mutex_;
    // rejected/withdrawn splices are moved to retired_splices so RT accesses splice
    // lock-free without use-after-free or ABA pointer reuse while this execution exists.
    std::shared_ptr<SpliceExecution> splice_owner;
    std::vector<std::shared_ptr<SpliceExecution>> retired_splices;
    std::atomic<SpliceExecution *> splice{nullptr};
    bool splice_reserved = false;  // Non-RT, under trajectory_mutex_.
    std::atomic<bool> on_tail{false};
    // Control thread only: adopted splice tail and per-candidate resolution tracking.
    SpliceExecution * tail = nullptr;
    const SpliceExecution * last_splice_seen = nullptr;
    const SpliceExecution * last_splice_resolved = nullptr;
    std::array<std::atomic<double>, 6> actual_pos{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> actual_vel{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> actual_eff{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> error_pos{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<std::atomic<double>, 6> error_vel{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // Optional per-goal tolerance overrides populated before RT handoff.
    std::array<double, 6> path_pos_tol{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> path_vel_tol{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> goal_pos_tol{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> goal_vel_tol{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    double goal_time_tol = 0.0;
    // Only the control thread accesses these playback fields.
    size_t last_sample_index = 0;
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
  bool valid_pva_sample_and_step(
    size_t joint_idx,
    double q, double dq, double ddq,
    double pos_limit_tol,
    double max_vel,
    double max_acc,
    bool has_prev,
    double prev_q, double prev_dq, double prev_ddq,
    double dt) const;
  bool valid_trajectory_goal(
    const FollowTrajectory::Goal & goal, bool * out_uniform_1khz = nullptr) const;
  bool check_trajectory_entry_continuity(
    const FollowTrajectory::Goal & goal,
    const std::array<double, 6> & q_meas,
    const std::array<double, 6> & dq_meas,
    int * out_joint = nullptr,
    double * out_value = nullptr,
    double * out_limit = nullptr) const;
  // Non-RT, under trajectory_mutex_: validates a SpliceJointTrajectory request.
  uint8_t valid_splice_request(
    const SpliceTrajectory::Request & req,
    bool * out_uniform_1khz = nullptr,
    std::string * out_message = nullptr) const;
  void populate_goal_tolerances(
    const FollowTrajectory::Goal & goal, TrajectoryExecution & exec) const;
  void publish_measured_state_atomic(bool valid);
  bool read_measured_state_atomic(
    std::array<double, 6> & q_out, std::array<double, 6> & dq_out) const;
  void monitor_trajectory();
  void check_trajectory_path();
  void finish_trajectory_cycle(double dt);
  void fail_trajectory(TrajectoryResult reason, int joint = -1,
    double value = 0.0, double limit = 0.0);
  controller_interface::return_type fail_controller(TrajectoryResult reason, int joint = -1);
  void request_execution_stop();
  void latch_estop_reason(uint8_t reason, int joint = -1, double value = 0.0, double limit = 0.0);
  void clear_estop_reason();
  void publish_shadow_shm(const Vec6d * target, bool valid);
  // Borrowed from the RT buffer once per update; valid until the next RT read.
  TrajectoryExecution * current_execution_ = nullptr;
  std::atomic<const TrajectoryExecution *> rt_seen_execution_{nullptr};
  static constexpr size_t kMaxTrajectoryPoints = 300001;
  std::vector<double> execution_max_joint_velocity_, execution_max_joint_acceleration_;
  double execution_position_limit_tolerance_ = 1e-4;
  std::vector<double> path_position_tolerance_, path_velocity_tolerance_;
  std::vector<double> goal_position_tolerance_, goal_velocity_tolerance_;
  std::vector<double> sysid_goal_position_tolerance_, sysid_goal_velocity_tolerance_;
  double entry_position_tolerance_ = 0.05;
  double entry_velocity_tolerance_ = 0.05;
  double goal_settle_duration_ = 1.0;
  double goal_time_tolerance_ = 5.0;
  double saturation_timeout_ = 1.0;
  // Lock-free seqlock snapshot of measured joint state for non-RT goal admission.
  std::atomic<uint64_t> measured_state_seq_{0};
  std::atomic<bool> measured_state_valid_{false};
  std::array<std::atomic<double>, 6> measured_q_atomic_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<std::atomic<double>, 6> measured_dq_atomic_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<bool, 6> effort_saturated_{};
  std::vector<double> reference_interfaces_;
  Vec6d tau_point_ff_ = Vec6d::Zero();
  std::atomic<bool> trajectory_enabled_{false};
  std::mutex trajectory_mutex_;  // Non-real-time callbacks only.
  bool trajectory_reserved_ = false;
  std::shared_ptr<TrajectoryExecution> active_trajectory_;
  std::vector<std::shared_ptr<TrajectoryExecution>> retired_trajectories_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajectoryExecution>> trajectory_buffer_;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr trajectory_server_;
  rclcpp::Service<SpliceTrajectory>::SharedPtr splice_service_;
  rclcpp::TimerBase::SharedPtr trajectory_monitor_;

  std::vector<std::string> joint_names_;

  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;
  pinocchio::JointIndex joint6_id_ = 0;
  pinocchio::Inertia base_joint6_inertia_ = pinocchio::Inertia::Zero();
  pinocchio::SE3 eef_to_joint6_placement_ = pinocchio::SE3::Identity();
  // Full articulated dynamics; command/reference/estop vectors stay six-axis.
  Vec8d dynamics_q_ = Vec8d::Zero(), dynamics_dq_ = Vec8d::Zero(), dynamics_ddq_ = Vec8d::Zero();

  Vec6d e_ = Vec6d::Zero(), de_ = Vec6d::Zero(), a_d_ = Vec6d::Zero();
  Vec6d tau_fric_ = Vec6d::Zero(), tau_cmd_ = Vec6d::Zero();
  Vec6d q_target_ = Vec6d::Zero(), dq_target_ = Vec6d::Zero(), ddq_target_ = Vec6d::Zero();

  Vec6d q_ = Vec6d::Zero(), dq_ = Vec6d::Zero();

  // Diagonal PD control gains
  Vec6d kp_ = Vec6d::Zero(), kv_ = Vec6d::Zero();

  // Torque-domain integral: gains in Nm/(rad s), back-calculation in 1/s,
  // and both the retained state and its limits in Nm.
  Vec6d ki_ = Vec6d::Zero(), antiwindup_gain_ = Vec6d::Zero();
  Vec6d integral_max_effort_ = Vec6d::Zero(), integral_effort_ = Vec6d::Zero();
  Vec6d friction_v_ = Vec6d::Zero(), friction_c_ = Vec6d::Zero(), armature_ = Vec6d::Zero();
  Vec6d max_effort_ = Vec6d::Zero();
  double tanh_slope_ = 300.0;

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

  struct ComplianceModeConfig {
    double max_joint_velocity = 3.0;
    double hard_limit_tolerance = 0.01;
    double saturation_timeout = 1.0;
    double entry_max_speed = 0.03;
    double filter_time_constant = 0.15;
    double soft_limit_margin = 0.15;
    double soft_limit_stiffness = 80.0;
    double soft_limit_damping = 5.0;
    double torque_slew_rate = 120.0;
    double transition_duration = 0.25;
    double min_translation_damping = 2.0;
    double min_rotation_damping = 0.15;
    double min_effective_mass = 0.30;
    double max_effective_mass = 15.0;
    double min_effective_inertia = 0.005;
    double max_effective_inertia = 0.60;
    double operational_damping_lambda = 0.04;
  };

  bool compliance_active() const;
  bool impedance_trajectory_active() const;
  bool rigid_trajectory_active() const;
  const ComplianceModeConfig & active_compliance_config() const;
  bool configure_compliance();
  void compute_compliance(double dt);
  void compute_cartesian_effective_inertia(
    const Mat6d & j_arm, double lambda,
    double min_mass, double max_mass,
    double min_inertia, double max_inertia,
    Vec6d & out_lambda_diag) const;

  // Real-time payload inertia update and linear static-moment smoothing.
  void update_payload_model(double dt);
  double payload_filter_time_constant_ = 0.03;
  double payload_nominal_gyration_radius_ = 0.025;
  double payload_mass_ = 0.0;
  std::array<double, 3> payload_com_tcp_{0.0, 0.0, 0.0};
  double payload_mass_filtered_ = 0.0;
  Eigen::Vector3d payload_first_moment_link6_filtered_ = Eigen::Vector3d::Zero();
  bool payload_smoothing_active_ = false;

  // Passive M-weighted singularity damping:
  //   tau = -d0 * (M - J^T Lambda_reg J) * dq_err
  //       = -d0 * M * (M + J^T Sigma_lambda^-1 J)^-1 * M * dq_err
  // Symmetric positive definite everywhere and vanishing away from singularities,
  // using cached M(q).
  Vec6d compute_nullspace_damping_torque(
    const Mat6d & j_arm, double lambda, double nullspace_damping,
    const Vec6d & dq_err) const;
  // Transition-only torque slew-rate limiter.
  void begin_torque_transition(double duration_sec);
  void apply_transition_torque_slew(double dt, double slew_rate, Vec6d & tau);
  // Recapture the current measured pose as a stationary NORMAL reference and
  // reconcile integral_effort_ (plus a smooth cosine recovery bias for any load
  // exceeding integral_max_effort_) so the mode switch preserves tau_applied_prev_
  // continuously without an impulsive step kick.
  void bumpless_handover_to_normal(double transition_duration_sec);
  bool transition_slew_active_ = false;
  double transition_slew_elapsed_ = 0.0;
  double transition_slew_duration_ = 0.25;
  int transition_slew_converged_cycles_ = 0;
  Vec6d transition_last_tau_ = Vec6d::Zero();
  bool recovery_bias_active_ = false;
  double recovery_bias_elapsed_ = 0.0;
  double recovery_bias_duration_ = 0.20;
  Vec6d recovery_bias_init_ = Vec6d::Zero();

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
    GOAL_TIMEOUT,
    MODE_CHANGED,
    EPOCH_MISMATCH,
    STOPPING_TIMEOUT,
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
    // Populated during non-RT validation before RT handoff.
    bool uniform_1khz = true;
    double t0_sec = 0.0;
    // Control thread only fields.
    size_t last_sample_index = 0;
    double elapsed = 0.0;
    int64_t start_time_ns = 0;
    bool started = false;
    bool endpoint_applied = false;
    bool cancel_applied = false;
    double settled_duration = 0.0;
    bool was_settled = false;
    double saturation_duration = 0.0;
    int failure_joint = -1;
    double failure_value = 0.0;
    double failure_limit = 0.0;
    const char * failure_detail = nullptr;
  };
  bool configure_impedance_trajectory();
  bool valid_impedance_trajectory_goal(
    const FollowImpedance::Goal & goal, bool * out_uniform_1khz = nullptr) const;
  void update_impedance_trajectory_reference(const rclcpp::Time & time, const rclcpp::Duration & period);
  void compute_impedance_trajectory(double dt);
  void check_impedance_trajectory_path();
  void finish_impedance_trajectory_cycle(double dt);
  void fail_impedance_trajectory(ImpedanceTrajectoryResult reason, int joint = -1,
    double value = 0.0, double limit = 0.0, const char * detail = nullptr);
  void monitor_impedance_trajectory();

  ImpedanceTrajectoryExecution * current_impedance_execution_ = nullptr;
  std::atomic<const ImpedanceTrajectoryExecution *> rt_seen_impedance_execution_{nullptr};
  bool impedance_trajectory_reserved_ = false;
  std::shared_ptr<ImpedanceTrajectoryExecution> active_impedance_trajectory_;
  std::vector<std::shared_ptr<ImpedanceTrajectoryExecution>> retired_impedance_trajectories_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ImpedanceTrajectoryExecution>> impedance_trajectory_buffer_;
  rclcpp_action::Server<FollowImpedance>::SharedPtr impedance_trajectory_server_;

  pinocchio::Data pin_data_ref_;
  Vec8d dynamics_q_ref_ = Vec8d::Zero();
  Vec8d dynamics_dq_ref_ = Vec8d::Zero();
  Mat6x8d imp_ref_jacobian_ = Mat6x8d::Zero();
  Vec6d q_imp_ref_ = Vec6d::Zero();
  Vec6d dq_imp_ref_ = Vec6d::Zero();
  Vec6d ddq_imp_ref_ = Vec6d::Zero();
  bool imp_ref_kinematics_cached_ = false;
  Vec6d imp_ref_cached_q_ = Vec6d::Zero();
  Vec6d imp_ref_cached_dq_ = Vec6d::Zero();
  pinocchio::SE3 imp_ref_pose_cached_ = pinocchio::SE3::Identity();
  Vec6d imp_ref_vel_cached_ = Vec6d::Zero();
  Eigen::Vector3d imp_kt_target_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d imp_dt_target_ = Eigen::Vector3d::Constant(0.8);
  Eigen::Vector3d imp_kr_target_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d imp_dr_target_ = Eigen::Vector3d::Constant(0.8);
  double imp_null_damping_target_ = 0.15;
  Vec6d imp_wrench_ff_target_ = Vec6d::Zero();
  Eigen::Vector3d imp_kt_filtered_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d imp_dt_filtered_ = Eigen::Vector3d::Constant(0.8);
  Eigen::Vector3d imp_kr_filtered_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d imp_dr_filtered_ = Eigen::Vector3d::Constant(0.8);
  double imp_null_damping_filtered_ = 0.15;
  Vec6d imp_wrench_ff_filtered_ = Vec6d::Zero();
  double imp_active_transition_time_ = 0.30;
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
  double imp_goal_velocity_tolerance_ = 0.015;
  double imp_goal_settle_duration_ = 0.3;
  double imp_goal_time_tolerance_ = 5.0;
  double imp_entry_max_speed_ = 0.03;
  double imp_entry_position_tolerance_ = 0.15;
  double imp_dynamics_ff_scale_ = 1.0;
  double imp_fv_scale_ = 0.90;
  double imp_fc_scale_ = 0.80;
  double imp_fric_trans_speed_ = 0.025;
  double imp_fric_quiet_speed_ = 0.008;
  double imp_filter_time_constant_ = 0.10;
  double imp_min_translation_damping_ = 2.0;
  double imp_min_rotation_damping_ = 0.15;
  double imp_min_effective_mass_ = 0.30;
  double imp_max_effective_mass_ = 15.0;
  double imp_min_effective_inertia_ = 0.005;
  double imp_max_effective_inertia_ = 0.60;
  double imp_operational_damping_lambda_ = 0.04;
  double imp_soft_limit_margin_ = 0.15;
  double imp_soft_limit_stiffness_ = 80.0;
  double imp_soft_limit_damping_ = 5.0;
  double imp_torque_slew_rate_ = 150.0;

  ComplianceModeConfig joint_imp_cfg_;
  ComplianceModeConfig cart_imp_cfg_;
  ComplianceModeConfig teach_cfg_;
  double teach_armature_weight_ = 0.04;
  std::array<double, 6> teach_min_joint_damping_{0.15, 0.15, 0.10, 0.04, 0.03, 0.015};
  std::array<double, 6> teach_max_joint_damping_{15.0, 15.0, 10.0, 4.0, 3.0, 1.5};
  std::array<double, 5> teach_damping_levels_{22.0, 17.5, 13.5, 11.5, 9.5};

  // Shared dynamics and TCP kinematics cache for compliance and impedance control.
  void ensure_dynamics_cached(bool need_coriolis = false);
  void read_cached_tcp_kinematics(
    const pinocchio::SE3 * & out_pose, const Mat6x8d * & out_jacobian);
  void add_trajectory_friction_ff(
    double fv_scale, double fc_scale, double v_trans, double quiet_speed,
    const Vec6d & dq_ref, Vec6d & tau_out) const;
  static Eigen::Vector3d rotational_spring_torque(
    const Eigen::Matrix3d & error_rotation, const Eigen::Vector3d & stiffness);
  bool dynamics_cached_this_cycle_ = false;
  bool coriolis_cached_this_cycle_ = false;
  bool tcp_cached_this_cycle_ = false;
  pinocchio::Data pin_data_dynamics_;
  Mat6x8d tcp_jacobian_ = Mat6x8d::Zero();
  double cartesian_characteristic_length_ = 0.25;
  Vec6d tau_applied_prev_ = Vec6d::Zero();
  bool has_applied_tau_ = false;

  lite6_interfaces::msg::ComplianceSettings compliance_settings_, compliance_filtered_;
  // Per-axis gains [LOCAL_WORLD_ALIGNED] and feedforward wrench used by
  // MODE_CARTESIAN_IMPEDANCE. Commands broadcast the scalar ComplianceSettings; an
  // END_HOLD_CARTESIAN_IMPEDANCE handover keeps the trajectory's anisotropic gains and
  // final wrench_ff, starting from its filtered values for a continuous torque.
  void set_cartesian_targets_from_settings(bool reset_filtered);
  Eigen::Vector3d cart_kt_target_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d cart_dt_target_ = Eigen::Vector3d::Constant(0.8);
  Eigen::Vector3d cart_kr_target_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d cart_dr_target_ = Eigen::Vector3d::Constant(0.8);
  Eigen::Vector3d cart_kt_filtered_ = Eigen::Vector3d::Constant(200.0);
  Eigen::Vector3d cart_dt_filtered_ = Eigen::Vector3d::Constant(0.8);
  Eigen::Vector3d cart_kr_filtered_ = Eigen::Vector3d::Constant(8.0);
  Eigen::Vector3d cart_dr_filtered_ = Eigen::Vector3d::Constant(0.8);
  Vec6d cart_wrench_ff_target_ = Vec6d::Zero();
  Vec6d cart_wrench_ff_filtered_ = Vec6d::Zero();
  Vec6d compliance_q_ = Vec6d::Zero();
  pinocchio::SE3 compliance_pose_;
  pinocchio::FrameIndex compliance_frame_ = 0;
  double compliance_saturation_time_ = 0.0;
  std::atomic<bool> compliance_blocks_trajectory_{false};
  std::atomic<bool> emergency_pending_{false};
  SystemState system_state_ = SystemState::NORMAL;
  // Set by the first on_activate(); every later activation starts latched in ESTOP.
  bool activated_once_ = false;

  // Kinematic Deceleration Tracker
  Vec6d estop_q_ = Vec6d::Zero(), estop_dq_ = Vec6d::Zero();
  Vec6d estop_decel_ = Vec6d::Zero();

  struct PendingControllerCommand {
    uint64_t cmd_seq = 0;
    // Active controller incarnation captured when the command was queued;
    // verified at RT application so no command can cross an activation boundary.
    uint64_t controller_epoch = 0;
    // E-STOP generation observed when the service request arrived. A command
    // received before a later E-STOP request is rejected instead of applied after it.
    uint64_t estop_generation = 0;
    uint8_t command = 0;
    bool compliance_recapture = false;
    double joint_stiffness = 20.0;
    double joint_damping = 3.0;
    double translation_stiffness = 200.0;
    double translation_damping = 0.8;
    double rotation_stiffness = 8.0;
    double rotation_damping = 0.8;
    double sensitivity = 3.0;
    double nullspace_damping = 0.5;
    double payload_mass = 0.0;
    double payload_com_tcp[3]{0.0, 0.0, 0.0};
  };

  void configure_command_service();
  static uint8_t controller_mode_for_state(SystemState state);
  uint8_t current_mode_code() const;
  void transition_system_state(SystemState next_state);
  void process_controller_commands(double dt);
  uint8_t apply_controller_command(const PendingControllerCommand & cmd);
  void step_compliance_stopping(double dt);
  void check_compliance_safety_limits();
  void begin_deceleration_tracker();
  bool step_deceleration_tracker(double dt);

  rclcpp::Clock::SharedPtr node_clock_;
  // Serializes non-E-STOP commands: the single producer of cmd_ring_.
  std::mutex cmd_service_mutex_;
  // Leaf mutex (never held across waits) synchronizing non-RT enqueue with
  // on_activate/on_deactivate epoch and queue resets.
  std::mutex cmd_queue_mutex_;
  // Bounds concurrent non-E-STOP executor waiters so executor threads remain free for
  // lock-free CMD_ESTOP preemption and action/timer callbacks under command bursts.
  std::atomic<uint32_t> ordinary_cmd_waiters_{0};
  uint32_t max_ordinary_cmd_waiters_ = 4;
  // Reentrant so CMD_ESTOP never waits behind a pending command's RT handshake.
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::Service<lite6_interfaces::srv::ControllerCommand>::SharedPtr command_service_;
  lite6_interfaces::shm::ShmSpscRing<PendingControllerCommand, 32> cmd_ring_;
  std::atomic<uint64_t> command_sequence_{0};
  // Claim gate for the single queued command: holds its cmd_seq while it may be applied.
  // The RT loop and a timed-out service call race on a CAS to 0; exactly one wins, so
  // ACK_RT_TIMEOUT always means the command was withdrawn and will never be applied.
  std::atomic<uint64_t> cmd_gate_{0};
  // Highest cmd_seq definitively discarded during on_activate before RT could claim it.
  std::atomic<uint64_t> last_cmd_discarded_seq_{0};
  // Incremented by every CMD_ESTOP request (fast path, bypasses cmd_ring_).
  std::atomic<uint64_t> estop_generation_{0};
  // Highest cmd_seq assigned to a fast-path CMD_ESTOP so RT preserves sequence correlation.
  std::atomic<uint64_t> estop_latest_cmd_seq_{0};
  // Highest estop_generation_ acknowledged and latched by the 1 kHz RT loop.
  std::atomic<uint64_t> estop_rt_ack_generation_{0};
  std::atomic<uint64_t> controller_epoch_atomic_{0};
  std::atomic<uint64_t> mode_epoch_atomic_{0};
  std::atomic<uint8_t> current_mode_atomic_{lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN};
  std::atomic<uint64_t> last_cmd_ack_seq_{0};
  std::atomic<uint64_t> last_cmd_ack_controller_epoch_{0};
  std::atomic<uint8_t> last_cmd_ack_code_{lite6_interfaces::srv::ControllerCommand::Request::CMD_NONE};
  std::atomic<uint8_t> last_cmd_ack_result_{lite6_interfaces::srv::ControllerCommand::Response::ACK_NONE};
  std::atomic<uint8_t> last_cmd_ack_mode_{lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN};
  std::atomic<uint64_t> last_cmd_ack_mode_epoch_{0};

  uint64_t controller_epoch_ = 0;
  uint64_t mode_epoch_ = 0;
  uint64_t status_seq_ = 0;
  uint64_t last_cmd_seq_ = 0;
  uint8_t last_cmd_code_ = lite6_interfaces::srv::ControllerCommand::Request::CMD_NONE;
  uint8_t last_cmd_result_ = lite6_interfaces::srv::ControllerCommand::Response::ACK_NONE;
  double compliance_stopping_elapsed_ = 0.0;
  bool rigid_cancel_braking_ = false;
  double rigid_cancel_elapsed_ = 0.0;

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
