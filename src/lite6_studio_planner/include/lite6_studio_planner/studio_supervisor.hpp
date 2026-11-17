#ifndef LITE6_STUDIO_PLANNER__STUDIO_SUPERVISOR_HPP_
#define LITE6_STUDIO_PLANNER__STUDIO_SUPERVISOR_HPP_

#include "lite6_interfaces/msg/controller_status.hpp"
#include "lite6_interfaces/msg/gripper_state.hpp"
#include "lite6_interfaces/msg/studio_state.hpp"
#include "lite6_interfaces/srv/studio_command.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

namespace lite6_studio_planner
{

class StudioSupervisor
{
public:
  using Clock = std::chrono::steady_clock;
  using S = lite6_interfaces::msg::StudioState;
  using C = lite6_interfaces::srv::StudioCommand;
  using CS = lite6_interfaces::msg::ControllerStatus;
  using GS = lite6_interfaces::msg::GripperState;

  enum class RecoverySubPhase : uint8_t
  {
    NONE = 0,
    WAIT_SETTLE = 1,
    ENABLE_NORMAL = 2,
    POST_SETTLE = 3
  };

  struct Effects
  {
    bool cancel_controller_goals = false;
    bool send_estop_cmd = false;
    bool send_enter_normal_cmd = false;
    bool wake_worker = false;
  };

  struct CommandResult
  {
    bool accepted = false;
    uint64_t revision = 0;
    uint8_t state = S::STATE_BOOT_INIT;
    uint64_t controller_epoch = 0;
    std::string message;
    Effects effects;
  };

  struct Snapshot
  {
    uint64_t revision = 0;
    uint8_t state = S::STATE_BOOT_INIT;
    std::string loaded_program_id;
    int32_t current_line_number = 0;
    int32_t current_statement_index = -1;
    int32_t current_block_index = 0;
    int32_t total_blocks = 0;
    double block_progress_percent = 0.0;
    double total_progress_percent = 0.0;
    uint8_t controller_mode = CS::MODE_UNKNOWN;
    bool controller_fresh = false;
    bool joint_states_fresh = false;
    uint8_t gripper_state = GS::BLOCKED;
    bool gripper_ready = false;
    bool estop_latched = false;
    uint8_t estop_reason = CS::ESTOP_NONE;
    int32_t last_error_code = 0;
    std::array<double, 3> cartesian_pos_error_m{};
    std::array<double, 3> cartesian_rot_error_rad{};
    std::array<double, 6> external_joint_torque{};
    std::array<double, 6> external_cartesian_wrench{};
    bool contact_detected = false;
    std::string detail;
    uint64_t controller_epoch = 0;
    uint64_t program_revision = 0;
    uint64_t last_compile_request_id = 0;
  };

  explicit StudioSupervisor(std::function<void(const Snapshot &)> on_change = {});

  Snapshot snapshot() const;

  // Sensor & controller observers (capture Clock::now() under mutex_ for monotonic ordering)
  Effects observe_controller(const CS & status);
  Effects observe_joint_state(bool positions_valid, double max_abs_dq, bool dq_valid);
  void observe_gripper(uint8_t gripper_state, bool ready);
  Effects watchdog();

  // Compile coordination
  bool can_compile(std::string & out_reason) const;
  void on_compile_dry_run(uint64_t request_id);
  void on_compile_succeeded(
    const std::string & program_id,
    int32_t total_blocks,
    uint64_t program_revision,
    uint64_t request_id);
  void on_compile_failed(uint64_t request_id);

  // Command handling
  CommandResult handle_command(uint8_t command, uint64_t expected_revision, const std::string & reason);

  // Worker synchronization queries & callbacks (session-checked)
  uint64_t active_session() const;
  uint8_t current_state() const;
  std::pair<uint8_t, uint64_t> state_and_session() const;
  RecoverySubPhase recovery_subphase() const;
  bool stop_requested(uint64_t session) const;
  bool pause_requested(uint64_t session) const;
  bool is_step_mode(uint64_t session) const;
  bool is_fresh_joint() const;

  void update_execution_progress(
    uint64_t session,
    int32_t line_number,
    int32_t statement_index,
    int32_t block_index,
    double block_progress_pct,
    double total_progress_pct);
  void update_impedance_errors(
    uint64_t session,
    const std::array<double, 3> & pos_err_m,
    const std::array<double, 3> & rot_err_rad);
  void clear_impedance_errors();

  void on_pause_settled(
    uint64_t session,
    int32_t line_number,
    int32_t statement_index,
    int32_t block_index,
    double block_progress_pct,
    double total_progress_pct);

  void on_step_block_done(
    uint64_t session,
    bool more_blocks_remaining,
    int32_t next_block_idx,
    int32_t next_line_number,
    int32_t next_statement_index,
    double total_progress_pct);

  void on_program_completed(uint64_t session);
  void on_stop_completed(uint64_t session);
  void on_execution_fault(
    uint64_t session,
    int32_t error_code,
    const std::string & detail,
    bool trigger_estop,
    Effects & out_effects);

  void on_recovery_failed(uint64_t session, const std::string & reason);

private:
  bool fresh_joint_unlocked(Clock::time_point now) const;
  bool fresh_controller_unlocked(Clock::time_point now) const;
  bool settled_unlocked(Clock::time_point now) const;
  void reset_settle_unlocked();
  void notify_unlocked(const std::string & detail);
  Snapshot make_snapshot_unlocked(Clock::time_point now) const;
  void enter_estop_unlocked(const std::string & detail, uint8_t reason, Effects & effects);
  void enter_fault_unlocked(int32_t code, const std::string & detail);
  void finish_stop_unlocked(const std::string & detail);

  static std::string describe_estop_reason(uint8_t reason);

  mutable std::mutex mutex_;
  std::function<void(const Snapshot &)> on_change_;

  Snapshot snap_;
  uint64_t session_id_ = 1;
  bool has_program_ = false;
  bool stop_requested_ = false;
  bool pause_requested_ = false;
  bool step_mode_ = false;

  RecoverySubPhase recovery_phase_ = RecoverySubPhase::NONE;
  Clock::time_point phase_deadline_{};
  Clock::time_point post_settle_start_{};

  Clock::time_point joint_stamp_{};
  Clock::time_point controller_stamp_{};
  Clock::time_point gripper_stamp_{};

  bool settling_ = false;
  bool last_dq_valid_ = false;
  double last_max_abs_dq_ = 0.0;
  uint64_t last_controller_epoch_ = 0;
  uint64_t last_status_seq_ = 0;
  Clock::time_point settle_start_{};
  Clock::time_point last_settle_obs_{};
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__STUDIO_SUPERVISOR_HPP_
