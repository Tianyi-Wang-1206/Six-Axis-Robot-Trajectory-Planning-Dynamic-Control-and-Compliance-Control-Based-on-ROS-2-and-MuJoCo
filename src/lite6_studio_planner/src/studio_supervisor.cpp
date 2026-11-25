#include "lite6_studio_planner/studio_supervisor.hpp"
#include "lite6_interfaces/motion_contract.hpp"
#include "lite6_interfaces/shm_registers.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace lite6_studio_planner
{
namespace
{
constexpr auto kControllerObservationTimeout =
  std::chrono::nanoseconds(lite6_interfaces::contract::kControllerStatusStaleTimeoutNs);
constexpr auto kGripperTimeout = std::chrono::milliseconds(1500);
constexpr int64_t kSettleDurationNs = 300'000'000LL;
constexpr int64_t kSettleGapNs = 100'000'000LL;
constexpr int64_t kPostRecoverySettleNs = 200'000'000LL;
constexpr auto kRecoveryPhaseTimeout = std::chrono::seconds(8);
constexpr auto kStoppingTimeout = std::chrono::seconds(5);
constexpr auto kPauseTimeout = std::chrono::seconds(10);
// Bound for the controller to return to NORMAL after an accepted RESET_FAULT.
constexpr auto kFaultResetTimeout = std::chrono::seconds(3);
// Prolonged controller-status outage window after which monotonic epoch/seq filters reset
// so a restarted controller with a smaller epoch (e.g. cross-host or test restart) reconnects.
constexpr auto kControllerEpochResetTimeout = std::chrono::seconds(2);
// Minimum spacing between asynchronous watchdog CMD_ESTOP retries and guard window before
// allowing CMD_RECOVER while controller_mode is still MODE_NORMAL.
constexpr auto kEstopCmdRetryInterval = std::chrono::milliseconds(150);
constexpr auto kEstopCmdInFlightGuard = std::chrono::milliseconds(300);
// An RT fault (fail_controller) makes controller_manager deactivate the CTC; it then
// stops publishing status and no trajectory command can bring it back.
constexpr const char * kControllerLostHint =
  "No /ctc_controller/status heartbeat: ctc_controller may have been deactivated by "
  "controller_manager after an RT fault (non-finite feedback, effort write failure or "
  "update exception). Reactivate it with: ros2 service call "
  "/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "
  "\"{activate_controllers: [ctc_controller], strictness: 2}\" -- then RECOVER.";
constexpr double kSettleSpeedThreshold =
  lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec;

inline int64_t steady_ns(StudioSupervisor::Clock::time_point tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}
}  // namespace

StudioSupervisor::StudioSupervisor(std::function<void(const Snapshot &)> on_change)
: on_change_(std::move(on_change))
{
  const int64_t boot_ns = lite6_interfaces::shm::steady_now_ns();
  planner_epoch_ = static_cast<uint64_t>(boot_ns > 0 ? boot_ns : 1);
  snap_.planner_epoch = planner_epoch_;
  snap_.state = S::STATE_BOOT_INIT;
  snap_.detail = "Waiting for /joint_states and /ctc_controller/status";
}

bool StudioSupervisor::fresh_joint_unlocked(Clock::time_point now) const
{
  return lite6_interfaces::shm::is_stamp_fresh(
    joint_sample_ns_, steady_ns(now),
    lite6_interfaces::contract::kJointStateStaleTimeoutNs);
}

bool StudioSupervisor::fresh_controller_unlocked(Clock::time_point now) const
{
  return controller_stamp_ != Clock::time_point{} &&
         (now - controller_stamp_) <= kControllerObservationTimeout;
}

bool StudioSupervisor::settled_unlocked(Clock::time_point now) const
{
  const int64_t now_ns = steady_ns(now);
  return settling_ &&
         lite6_interfaces::shm::is_stamp_fresh(last_settle_obs_ns_, now_ns, kSettleGapNs) &&
         (last_settle_obs_ns_ - settle_start_ns_) >= kSettleDurationNs;
}

void StudioSupervisor::reset_settle_unlocked()
{
  settling_ = false;
  settle_start_ns_ = 0;
  last_settle_obs_ns_ = 0;
}

void StudioSupervisor::notify_unlocked(const std::string & detail)
{
  ++snap_.revision;
  snap_.detail = detail;
}

StudioSupervisor::Snapshot StudioSupervisor::make_snapshot_unlocked(Clock::time_point now) const
{
  Snapshot copy = snap_;
  copy.joint_states_fresh = fresh_joint_unlocked(now);
  copy.controller_fresh = fresh_controller_unlocked(now);
  const bool gripper_fresh =
    gripper_stamp_ != Clock::time_point{} && (now - gripper_stamp_) <= kGripperTimeout;
  copy.gripper_ready = gripper_fresh && copy.gripper_ready;
  return copy;
}

std::string StudioSupervisor::describe_estop_reason(uint8_t reason)
{
  switch (reason) {
    case CS::ESTOP_NONE: return "E-STOP active";
    case CS::ESTOP_EXTERNAL: return "External E-STOP";
    case CS::ESTOP_COMPLIANCE_POSITION_LIMIT: return "Compliance joint position limit exceeded";
    case CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT: return "Compliance joint velocity limit exceeded";
    case CS::ESTOP_COMPLIANCE_SATURATION: return "Compliance effort saturation timeout";
    case CS::ESTOP_TRAJECTORY_PATH_POSITION: return "Rigid trajectory path position tolerance violated";
    case CS::ESTOP_TRAJECTORY_PATH_VELOCITY: return "Rigid trajectory path velocity tolerance violated";
    case CS::ESTOP_TRAJECTORY_GOAL_TIMEOUT: return "Rigid trajectory settling deadline exceeded";
    case CS::ESTOP_TRAJECTORY_SATURATION: return "Rigid trajectory effort saturation timeout";
    case CS::ESTOP_TRAJECTORY_MODE_CHANGED: return "Controller mode changed mid-trajectory";
    case CS::ESTOP_INVALID_STATE: return "Non-finite joint feedback detected";
    case CS::ESTOP_INVALID_COMMAND: return "Non-finite control effort computed";
    case CS::ESTOP_COMMAND_WRITE: return "Effort interface write failure";
    case CS::ESTOP_CONTROL_EXCEPTION: return "Controller update exception";
    case CS::ESTOP_IMPEDANCE_TRAJECTORY_DEVIATION:
      return "Impedance trajectory deviation tolerance exceeded";
    case CS::ESTOP_IMPEDANCE_TRAJECTORY_GOAL_TIMEOUT:
      return "Impedance trajectory goal settling deadline exceeded";
    case CS::ESTOP_COMPLIANCE_STOPPING_TIMEOUT:
      return "Compliance/impedance exit deceleration timed out";
    default: return "Controller E-STOP latched";
  }
}

void StudioSupervisor::finish_stop_unlocked(const std::string & detail)
{
  ++session_id_;
  stop_requested_ = false;
  pause_requested_ = false;
  pause_deadline_ = Clock::time_point{};
  step_mode_ = false;
  snap_.current_block_index = 0;
  snap_.current_line_number = 0;
  snap_.current_statement_index = -1;
  snap_.block_progress_percent = 0.0;
  snap_.total_progress_percent = 0.0;
  snap_.cartesian_pos_error_m.fill(0.0);
  snap_.cartesian_rot_error_rad.fill(0.0);
  snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
  notify_unlocked(detail);
}

void StudioSupervisor::enter_estop_unlocked(
  const std::string & detail, uint8_t reason, Effects & effects)
{
  ++session_id_;
  stop_requested_ = false;
  pause_requested_ = false;
  pause_deadline_ = Clock::time_point{};
  step_mode_ = false;
  recovery_phase_ = RecoverySubPhase::NONE;
  recovery_initial_mode_ = CS::MODE_UNKNOWN;
  recovery_initial_mode_epoch_ = 0;
  fault_reset_pending_ = false;
  const bool was_latched = snap_.estop_latched;
  snap_.estop_latched = true;
  if (reason != CS::ESTOP_NONE) {
    // Preserve an already-latched specific root-cause reason (e.g. PATH_POSITION or
    // SATURATION) when a redundant operator/external CMD_ESTOP arrives.
    if (!was_latched || snap_.estop_reason == CS::ESTOP_NONE ||
        snap_.estop_reason == CS::ESTOP_EXTERNAL || reason != CS::ESTOP_EXTERNAL)
    {
      snap_.estop_reason = reason;
    }
  } else if (snap_.estop_reason == CS::ESTOP_NONE) {
    snap_.estop_reason = CS::ESTOP_EXTERNAL;
  }
  snap_.state = S::STATE_ESTOP;
  snap_.current_block_index = 0;
  snap_.current_line_number = 0;
  snap_.current_statement_index = -1;
  snap_.block_progress_percent = 0.0;
  snap_.total_progress_percent = 0.0;
  snap_.cartesian_pos_error_m.fill(0.0);
  snap_.cartesian_rot_error_rad.fill(0.0);
  effects.cancel_controller_goals = true;
  effects.wake_worker = true;
  notify_unlocked(detail);
}

void StudioSupervisor::enter_fault_unlocked(int32_t code, const std::string & detail)
{
  ++session_id_;
  stop_requested_ = false;
  pause_requested_ = false;
  pause_deadline_ = Clock::time_point{};
  step_mode_ = false;
  recovery_phase_ = RecoverySubPhase::NONE;
  recovery_initial_mode_ = CS::MODE_UNKNOWN;
  recovery_initial_mode_epoch_ = 0;
  fault_reset_pending_ = false;
  snap_.last_error_code = code;
  snap_.state = S::STATE_FAULT;
  notify_unlocked(detail);
}

void StudioSupervisor::clear_fault_unlocked(const std::string & detail)
{
  fault_reset_pending_ = false;
  snap_.last_error_code = 0;
  snap_.current_block_index = 0;
  snap_.current_line_number = 0;
  snap_.current_statement_index = -1;
  snap_.block_progress_percent = 0.0;
  snap_.total_progress_percent = 0.0;
  snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
  notify_unlocked(detail);
}

StudioSupervisor::Snapshot StudioSupervisor::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return make_snapshot_unlocked(Clock::now());
}

StudioSupervisor::Effects StudioSupervisor::observe_controller(const CS & status)
{
  const uint8_t mode = status.mode;
  const uint8_t estop_reason = status.estop_reason;
  Effects effects;
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    // Reset monotonic epoch/seq filters after a prolonged controller disconnect so a
    // restarted controller with a smaller epoch (cross-host or mock restart) is accepted.
    if (controller_stamp_ != Clock::time_point{} &&
        (now - controller_stamp_) >= kControllerEpochResetTimeout)
    {
      last_controller_epoch_ = 0;
      last_status_seq_ = 0;
      last_mode_epoch_ = 0;
    }

    if (status.controller_epoch != 0 || status.status_seq != 0) {
      if (status.controller_epoch < last_controller_epoch_) {
        return effects;
      }
      if (status.controller_epoch == last_controller_epoch_ &&
          status.status_seq <= last_status_seq_)
      {
        return effects;
      }
      const bool epoch_changed =
        last_controller_epoch_ != 0 &&
        status.controller_epoch != 0 &&
        status.controller_epoch != last_controller_epoch_;
      last_controller_epoch_ = status.controller_epoch;
      last_status_seq_ = status.status_seq;
      last_mode_epoch_ = status.mode_epoch;
      snap_.controller_epoch = status.controller_epoch;
      if (epoch_changed &&
          (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING ||
           snap_.state == S::STATE_STOPPING || snap_.state == S::STATE_RECOVERING ||
           snap_.state == S::STATE_PAUSED))
      {
        effects.cancel_controller_goals = true;
        if (mode == CS::MODE_ESTOP) {
          enter_estop_unlocked(
            "Controller reactivated in ESTOP mid-operation (epoch changed)",
            estop_reason, effects);
        } else {
          enter_fault_unlocked(
            -12, "Controller reactivated mid-operation (controller_epoch changed)");
        }
        changed = true;
      }
    } else if (status.mode_epoch != 0) {
      last_mode_epoch_ = status.mode_epoch;
    }

    const bool was_fresh = fresh_controller_unlocked(now);
    const bool was_lost_notified = controller_lost_notified_;
    const uint8_t prev_mode = snap_.controller_mode;
    controller_stamp_ = now;
    controller_lost_notified_ = false;
    if (was_lost_notified) {
      const std::string suffix = std::string("\n\n") + kControllerLostHint;
      if (snap_.detail.size() >= suffix.size() &&
          snap_.detail.compare(snap_.detail.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        snap_.detail.erase(snap_.detail.size() - suffix.size());
      }
    }
    snap_.controller_mode = mode;
    snap_.last_cmd_seq = status.last_cmd_seq;
    snap_.last_cmd_code = status.last_cmd_code;
    snap_.last_cmd_result = status.last_cmd_result;
    snap_.payload_mass = status.payload_mass;
    snap_.payload_com_tcp = status.payload_com_tcp;

    if (changed) {
      notify_snap = make_snapshot_unlocked(now);
    } else if (mode == CS::MODE_ESTOP) {
      const bool was_latched = snap_.estop_latched;
      snap_.estop_latched = true;
      if (estop_reason != CS::ESTOP_NONE &&
          (!was_latched || snap_.estop_reason == CS::ESTOP_NONE ||
           snap_.estop_reason == CS::ESTOP_EXTERNAL || estop_reason != CS::ESTOP_EXTERNAL))
      {
        snap_.estop_reason = estop_reason;
      }
      if (snap_.state != S::STATE_ESTOP && snap_.state != S::STATE_RECOVERING) {
        enter_estop_unlocked(describe_estop_reason(snap_.estop_reason), snap_.estop_reason, effects);
        changed = true;
      } else if (snap_.state == S::STATE_RECOVERING) {
        const bool new_estop_episode =
          (recovery_phase_ == RecoverySubPhase::POST_SETTLE) ||
          (recovery_initial_mode_ != CS::MODE_ESTOP) ||
          (status.mode_epoch != 0 && recovery_initial_mode_epoch_ != 0 &&
           status.mode_epoch > recovery_initial_mode_epoch_);
        if (new_estop_episode) {
          enter_estop_unlocked(
            recovery_phase_ == RecoverySubPhase::POST_SETTLE ?
            "Controller re-entered E-STOP during recovery verification" :
            "New E-STOP latched during recovery; retry RECOVER",
            snap_.estop_reason, effects);
          changed = true;
        }
      }
    } else if (mode == CS::MODE_NORMAL) {
      if (snap_.state == S::STATE_BOOT_INIT) {
        if (fresh_joint_unlocked(now)) {
          snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
          notify_unlocked("Controller NORMAL and joint telemetry fresh; system ready");
          changed = true;
        }
      } else if (
        snap_.state == S::STATE_FAULT && fault_reset_pending_ && fresh_joint_unlocked(now))
      {
        clear_fault_unlocked("Controller returned to MODE_NORMAL; fault cleared");
        changed = true;
      } else if (
        snap_.state == S::STATE_RECOVERING &&
        recovery_phase_ == RecoverySubPhase::ENABLE_NORMAL)
      {
        recovery_phase_ = RecoverySubPhase::POST_SETTLE;
        post_settle_start_ns_ = steady_ns(now);
        phase_deadline_ = now + kRecoveryPhaseTimeout;
        notify_unlocked("Controller entered NORMAL; verifying stationary settle");
        changed = true;
      }
    } else if (
      mode == CS::MODE_CARTESIAN_IMPEDANCE_TRAJECTORY ||
      mode == CS::MODE_COMPLIANCE_STOPPING ||
      mode == CS::MODE_CARTESIAN_IMPEDANCE)
    {
      // Permitted only during active execution (RUNNING/STEPPING) or controlled stopping;
      // PAUSED and idle states require MODE_NORMAL so RESUME/STEP/STOP cannot trap or fault.
      if (snap_.state == S::STATE_BOOT_INIT || snap_.state == S::STATE_IDLE ||
          snap_.state == S::STATE_COMPILED_READY || snap_.state == S::STATE_PAUSED)
      {
        enter_fault_unlocked(
          -10, "Unexpected active compliance/impedance mode while controller must be in MODE_NORMAL");
        changed = true;
      } else if (snap_.state == S::STATE_RECOVERING) {
        effects.cancel_controller_goals = true;
        effects.send_estop_cmd = true;
        last_estop_cmd_sent_ = now;
        enter_estop_unlocked(
          "Unexpected compliance/impedance mode during E-STOP recovery",
          CS::ESTOP_TRAJECTORY_MODE_CHANGED, effects);
        changed = true;
      }
    } else {
      // Unexpected external controller mode (SYSID, TEACH, standalone joint impedance, UNKNOWN)
      if (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING ||
          snap_.state == S::STATE_STOPPING || snap_.state == S::STATE_RECOVERING)
      {
        effects.cancel_controller_goals = true;
        effects.send_estop_cmd = true;
        last_estop_cmd_sent_ = now;
        enter_estop_unlocked(
          "Unexpected controller mode during active operation",
          CS::ESTOP_TRAJECTORY_MODE_CHANGED, effects);
        changed = true;
      } else if (snap_.state != S::STATE_ESTOP && snap_.state != S::STATE_FAULT) {
        enter_fault_unlocked(-11, "Controller is in incompatible mode for trajectory execution");
        changed = true;
      }
    }

    if (!changed && (!was_fresh || was_lost_notified || prev_mode != mode)) {
      notify_unlocked(snap_.detail);
      changed = true;
    }
    if (changed) {
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
  return effects;
}

StudioSupervisor::Effects StudioSupervisor::observe_joint_state(
  bool positions_valid, double max_abs_dq, bool dq_valid, int64_t sample_steady_ns)
{
  Effects effects;
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    const int64_t now_ns = steady_ns(now);
    // Validate the source steady-clock timestamp against the 100 ms contract (also rejects
    // future or out-of-order older timestamps so freshness can never exceed the source age).
    if (!lite6_interfaces::shm::is_stamp_fresh(
          sample_steady_ns, now_ns, lite6_interfaces::contract::kJointStateStaleTimeoutNs) ||
        (joint_sample_ns_ > 0 && sample_steady_ns < joint_sample_ns_))
    {
      return effects;
    }

    if (!positions_valid || !dq_valid || !std::isfinite(max_abs_dq)) {
      reset_settle_unlocked();
      last_dq_valid_ = false;
      if (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING ||
          snap_.state == S::STATE_RECOVERING || snap_.state == S::STATE_STOPPING)
      {
        effects.send_estop_cmd = true;
        last_estop_cmd_sent_ = now;
        enter_estop_unlocked(
          "Invalid / non-finite /joint_states observation",
          CS::ESTOP_INVALID_STATE, effects);
        changed = true;
      }
      if (changed) {
        notify_snap = make_snapshot_unlocked(now);
      }
    } else {
      const bool was_fresh = fresh_joint_unlocked(now);
      joint_sample_ns_ = sample_steady_ns;
      last_dq_valid_ = true;
      last_max_abs_dq_ = max_abs_dq;

      if (max_abs_dq <= kSettleSpeedThreshold) {
        if (!settling_ ||
            !lite6_interfaces::shm::is_stamp_fresh(
              last_settle_obs_ns_, sample_steady_ns, kSettleGapNs))
        {
          settle_start_ns_ = sample_steady_ns;
          settling_ = true;
        }
        last_settle_obs_ns_ = sample_steady_ns;
      } else {
        reset_settle_unlocked();
      }

      if (snap_.state == S::STATE_BOOT_INIT) {
        if (fresh_controller_unlocked(now)) {
          if (snap_.controller_mode == CS::MODE_ESTOP) {
            enter_estop_unlocked(
              describe_estop_reason(snap_.estop_reason), snap_.estop_reason, effects);
            changed = true;
          } else if (snap_.controller_mode == CS::MODE_NORMAL) {
            snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
            notify_unlocked("Controller NORMAL and joint telemetry fresh; system ready");
            changed = true;
          }
        }
      } else if (snap_.state == S::STATE_RECOVERING) {
        if (recovery_phase_ == RecoverySubPhase::WAIT_SETTLE &&
            fresh_controller_unlocked(now) &&
            settled_unlocked(now))
        {
          if (snap_.controller_mode == CS::MODE_NORMAL) {
            recovery_phase_ = RecoverySubPhase::POST_SETTLE;
            post_settle_start_ns_ = now_ns;
            phase_deadline_ = now + kRecoveryPhaseTimeout;
            notify_unlocked("Robot settled and controller already NORMAL; finalizing recovery");
          } else {
            recovery_phase_ = RecoverySubPhase::ENABLE_NORMAL;
            phase_deadline_ = now + kRecoveryPhaseTimeout;
            effects.wake_worker = true;
            notify_unlocked("Robot settled; issuing CMD_CLEAR_ESTOP to controller");
          }
          changed = true;
        } else if (
          recovery_phase_ == RecoverySubPhase::POST_SETTLE &&
          snap_.controller_mode == CS::MODE_NORMAL &&
          fresh_controller_unlocked(now) &&
          settled_unlocked(now) &&
          last_settle_obs_ns_ >= post_settle_start_ns_ &&
          (now_ns - post_settle_start_ns_) >= kPostRecoverySettleNs)
        {
          recovery_phase_ = RecoverySubPhase::NONE;
          recovery_initial_mode_ = CS::MODE_UNKNOWN;
          recovery_initial_mode_epoch_ = 0;
          snap_.estop_latched = false;
          snap_.estop_reason = CS::ESTOP_NONE;
          snap_.last_error_code = 0;
          snap_.current_block_index = 0;
          snap_.current_line_number = 0;
          snap_.current_statement_index = -1;
          snap_.block_progress_percent = 0.0;
          snap_.total_progress_percent = 0.0;
          snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
          notify_unlocked("E-STOP recovery complete");
          changed = true;
        }
      }

      if (!changed && !was_fresh) {
        notify_unlocked(snap_.detail);
        changed = true;
      }
      if (changed) {
        notify_snap = make_snapshot_unlocked(now);
      }
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
  return effects;
}

void StudioSupervisor::observe_gripper(uint8_t gripper_state, bool ready)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    gripper_stamp_ = now;
    if (snap_.gripper_state != gripper_state || snap_.gripper_ready != ready) {
      snap_.gripper_state = gripper_state;
      snap_.gripper_ready = ready;
      changed = true;
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

StudioSupervisor::Effects StudioSupervisor::watchdog()
{
  Effects effects;
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    const bool j_fresh = fresh_joint_unlocked(now);
    const bool c_fresh = fresh_controller_unlocked(now);

    const bool controller_lost =
      controller_stamp_ != Clock::time_point{} && !c_fresh;
    if ((!j_fresh || !c_fresh) &&
        (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING ||
         snap_.state == S::STATE_STOPPING || snap_.state == S::STATE_RECOVERING))
    {
      effects.send_estop_cmd = true;
      last_estop_cmd_sent_ = now;
      std::string detail =
        "Feedback timeout on /joint_states or /ctc_controller/status during motion";
      if (controller_lost) {
        controller_lost_notified_ = true;
        detail += std::string("\n\n") + kControllerLostHint;
      }
      enter_estop_unlocked(detail, CS::ESTOP_EXTERNAL, effects);
      changed = true;
    } else if (controller_lost && !controller_lost_notified_ &&
               snap_.state != S::STATE_BOOT_INIT)
    {
      controller_lost_notified_ = true;
      notify_unlocked(snap_.detail + "\n\n" + kControllerLostHint);
      changed = true;
    } else if (snap_.state == S::STATE_ESTOP && c_fresh &&
               snap_.controller_mode != CS::MODE_ESTOP)
    {
      if (last_estop_cmd_sent_ == Clock::time_point{} ||
          (now - last_estop_cmd_sent_) >= kEstopCmdRetryInterval)
      {
        effects.send_estop_cmd = true;
        last_estop_cmd_sent_ = now;
      }
    } else if (snap_.state == S::STATE_RECOVERING) {
      if (recovery_phase_ == RecoverySubPhase::POST_SETTLE &&
          snap_.controller_mode == CS::MODE_NORMAL &&
          c_fresh && j_fresh &&
          settled_unlocked(now) &&
          last_settle_obs_ns_ >= post_settle_start_ns_ &&
          (steady_ns(now) - post_settle_start_ns_) >= kPostRecoverySettleNs)
      {
        recovery_phase_ = RecoverySubPhase::NONE;
        recovery_initial_mode_ = CS::MODE_UNKNOWN;
        recovery_initial_mode_epoch_ = 0;
        snap_.estop_latched = false;
        snap_.estop_reason = CS::ESTOP_NONE;
        snap_.last_error_code = 0;
        snap_.current_block_index = 0;
        snap_.current_line_number = 0;
        snap_.current_statement_index = -1;
        snap_.block_progress_percent = 0.0;
        snap_.total_progress_percent = 0.0;
        snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
        notify_unlocked("E-STOP recovery complete");
        changed = true;
      } else if (now > phase_deadline_) {
        enter_estop_unlocked(
          "E-STOP recovery timed out; verify robot is stationary and retry RECOVER",
          snap_.estop_reason, effects);
        changed = true;
      }
    } else if ((snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING) &&
               pause_requested_ &&
               pause_deadline_ != Clock::time_point{} &&
               now > pause_deadline_)
    {
      stop_requested_ = true;
      pause_requested_ = false;
      pause_deadline_ = Clock::time_point{};
      snap_.state = S::STATE_STOPPING;
      phase_deadline_ = now + kStoppingTimeout;
      effects.cancel_controller_goals = true;
      effects.wake_worker = true;
      notify_unlocked("Pause deadline exceeded; stopping execution cleanly");
      changed = true;
    } else if (snap_.state == S::STATE_STOPPING && now > phase_deadline_) {
      effects.send_estop_cmd = true;
      last_estop_cmd_sent_ = now;
      enter_estop_unlocked(
        "Controlled stop deadline exceeded; latched E-STOP",
        CS::ESTOP_EXTERNAL, effects);
      changed = true;
    } else if (snap_.state == S::STATE_FAULT && fault_reset_pending_ &&
               now > fault_reset_deadline_)
    {
      fault_reset_pending_ = false;
      notify_unlocked(
        "Automatic fault reset timed out: controller did not return to MODE_NORMAL; "
        "retry RESET_FAULT");
      changed = true;
    }

    if (changed) {
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
  return effects;
}

bool StudioSupervisor::begin_compile(bool auto_load, std::string & out_reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (auto_load) {
    switch (snap_.state) {
      case S::STATE_IDLE:
      case S::STATE_COMPILED_READY:
      case S::STATE_FAULT:
      case S::STATE_ESTOP:
        break;
      default:
        out_reason =
          "Cannot compile while system state is PAUSED or active "
          "(RUNNING/STEPPING/STOPPING/RECOVERING/BOOT_INIT); STOP program first";
        return false;
    }
    ++compile_in_flight_;
  }
  return true;
}

void StudioSupervisor::end_compile(bool auto_load)
{
  if (!auto_load) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (compile_in_flight_ > 0) {
    --compile_in_flight_;
  }
}

void StudioSupervisor::on_compile_dry_run(uint64_t request_id)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request_id != 0 && snap_.last_compile_request_id != request_id) {
      snap_.last_compile_request_id = request_id;
      notify_unlocked(snap_.detail);
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_compile_succeeded(
  const std::string & program_id,
  int32_t total_blocks,
  uint64_t program_revision,
  uint64_t request_id)
{
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_program_ = true;
    snap_.loaded_program_id = program_id;
    snap_.program_revision = program_revision;
    if (request_id != 0) {
      snap_.last_compile_request_id = request_id;
    }
    snap_.total_blocks = total_blocks;
    snap_.current_block_index = 0;
    snap_.current_line_number = 0;
    snap_.current_statement_index = -1;
    snap_.block_progress_percent = 0.0;
    snap_.total_progress_percent = 0.0;
    snap_.cartesian_pos_error_m.fill(0.0);
    snap_.cartesian_rot_error_rad.fill(0.0);
    if (snap_.state == S::STATE_IDLE || snap_.state == S::STATE_COMPILED_READY) {
      snap_.state = S::STATE_COMPILED_READY;
      notify_unlocked("Program compiled and ready (" + std::to_string(total_blocks) + " blocks)");
    } else {
      notify_unlocked(
        "Program compiled (" + std::to_string(total_blocks) +
        " blocks); clear fault or recover E-STOP before execution");
    }
    notify_snap = make_snapshot_unlocked(Clock::now());
  }
  if (on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_compile_failed(uint64_t request_id)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request_id != 0) {
      snap_.last_compile_request_id = request_id;
    }
    if (has_program_ || snap_.state == S::STATE_COMPILED_READY || snap_.state == S::STATE_PAUSED) {
      has_program_ = false;
      snap_.loaded_program_id.clear();
      snap_.program_revision = 0;
      snap_.total_blocks = 0;
      snap_.current_block_index = 0;
      snap_.current_line_number = 0;
      snap_.current_statement_index = -1;
      snap_.block_progress_percent = 0.0;
      snap_.total_progress_percent = 0.0;
      if (snap_.state == S::STATE_COMPILED_READY || snap_.state == S::STATE_PAUSED) {
        snap_.state = S::STATE_IDLE;
      }
      notify_unlocked("Compilation failed; previously loaded program cleared");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    } else if (request_id != 0) {
      notify_unlocked("Compilation failed");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

StudioSupervisor::CommandResult StudioSupervisor::handle_command(
  uint8_t command,
  uint64_t expected_planner_epoch,
  uint64_t expected_revision,
  const std::string & reason)
{
  CommandResult res;
  bool changed = false;
  Snapshot notify_snap;
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto reject = [&](const std::string & msg) {
      res.accepted = false;
      res.planner_epoch = planner_epoch_;
      res.revision = snap_.revision;
      res.state = snap_.state;
      res.controller_epoch = snap_.controller_epoch;
      res.message = msg;
    };

    if (command != C::Request::CMD_ESTOP &&
        expected_planner_epoch > 0 && expected_planner_epoch != planner_epoch_)
    {
      reject("Planner epoch mismatch: expected " + std::to_string(expected_planner_epoch) +
        ", current " + std::to_string(planner_epoch_));
      return res;
    }

    if (command != C::Request::CMD_ESTOP &&
        expected_revision > 0 && expected_revision != snap_.revision)
    {
      reject("Revision mismatch: expected " + std::to_string(expected_revision) +
        ", current " + std::to_string(snap_.revision));
      return res;
    }

    const bool j_fresh = fresh_joint_unlocked(now);
    const bool c_fresh = fresh_controller_unlocked(now);
    const bool stationary = last_dq_valid_ && (last_max_abs_dq_ <= kSettleSpeedThreshold);
    res.prev_state = snap_.state;

    if (compile_in_flight_ > 0 &&
        (command == C::Request::CMD_RUN || command == C::Request::CMD_RESUME ||
         command == C::Request::CMD_STEP || command == C::Request::CMD_CLEAR_PROGRAM))
    {
      reject("Compilation is currently in progress; wait for completion");
      return res;
    }

    switch (command) {
      case C::Request::CMD_ESTOP: {
        res.effects.send_estop_cmd = true;
        last_estop_cmd_sent_ = now;
        enter_estop_unlocked(
          reason.empty() ? "Operator commanded E-STOP" : ("E-STOP: " + reason),
          CS::ESTOP_EXTERNAL, res.effects);
        res.accepted = true;
        res.revision = snap_.revision;
        res.message = snap_.detail;
        changed = true;
        break;
      }

      case C::Request::CMD_RUN: {
        if (snap_.state == S::STATE_PAUSED) {
          reject("Program is PAUSED; use CMD_RESUME to continue or CMD_STOP before restarting");
          break;
        }
        if (snap_.state != S::STATE_COMPILED_READY || !has_program_) {
          reject("RUN requires STATE_COMPILED_READY with a valid compiled program");
          break;
        }
        if (!j_fresh || !c_fresh || snap_.estop_latched ||
            snap_.controller_mode != CS::MODE_NORMAL || !stationary)
        {
          reject("RUN requires fresh telemetry, stationary joints, no E-STOP latch, and controller in MODE_NORMAL");
          break;
        }
        ++session_id_;
        stop_requested_ = false;
        pause_requested_ = false;
        step_mode_ = false;
        snap_.current_block_index = 0;
        snap_.current_line_number = 0;
        snap_.current_statement_index = -1;
        snap_.block_progress_percent = 0.0;
        snap_.total_progress_percent = 0.0;
        snap_.last_error_code = 0;
        snap_.cartesian_pos_error_m.fill(0.0);
        snap_.cartesian_rot_error_rad.fill(0.0);
        snap_.state = S::STATE_RUNNING;
        notify_unlocked("Executing compiled program");
        res.accepted = true;
        res.effects.wake_worker = true;
        changed = true;
        break;
      }

      case C::Request::CMD_RESUME: {
        if (snap_.state != S::STATE_PAUSED || !has_program_) {
          reject("RESUME is only valid from STATE_PAUSED");
          break;
        }
        if (!j_fresh || !c_fresh || snap_.estop_latched ||
            snap_.controller_mode != CS::MODE_NORMAL || !stationary)
        {
          reject("RESUME requires fresh telemetry, stationary joints, and controller in MODE_NORMAL");
          break;
        }
        ++session_id_;
        stop_requested_ = false;
        pause_requested_ = false;
        step_mode_ = false;
        snap_.state = S::STATE_RUNNING;
        notify_unlocked("Resuming program execution");
        res.accepted = true;
        res.effects.wake_worker = true;
        changed = true;
        break;
      }

      case C::Request::CMD_STEP: {
        if ((snap_.state != S::STATE_COMPILED_READY && snap_.state != S::STATE_PAUSED) ||
            !has_program_)
        {
          reject("STEP requires STATE_COMPILED_READY or STATE_PAUSED");
          break;
        }
        if (!j_fresh || !c_fresh || snap_.estop_latched ||
            snap_.controller_mode != CS::MODE_NORMAL || !stationary)
        {
          reject("STEP requires fresh telemetry, stationary joints, and controller in MODE_NORMAL");
          break;
        }
        if (snap_.state == S::STATE_COMPILED_READY) {
          snap_.current_block_index = 0;
          snap_.block_progress_percent = 0.0;
          snap_.total_progress_percent = 0.0;
        }
        ++session_id_;
        stop_requested_ = false;
        pause_requested_ = false;
        step_mode_ = true;
        snap_.last_error_code = 0;
        snap_.state = S::STATE_STEPPING;
        notify_unlocked("Stepping single execution block");
        res.accepted = true;
        res.effects.wake_worker = true;
        changed = true;
        break;
      }

      case C::Request::CMD_PAUSE: {
        if (snap_.state != S::STATE_RUNNING && snap_.state != S::STATE_STEPPING) {
          reject("PAUSE is only valid while RUNNING or STEPPING");
          break;
        }
        if (pause_requested_) {
          reject("PAUSE already pending");
          break;
        }
        pause_requested_ = true;
        pause_deadline_ = now + kPauseTimeout;
        // No direct goal cancellation: the worker observes pause_requested_ within one
        // 20 ms poll and either splices a stop-along-path tail into a rigid block or
        // cancels impedance / alignment goals itself.
        notify_unlocked("Pause requested; decelerating to stationary pause state");
        res.accepted = true;
        changed = true;
        break;
      }

      case C::Request::CMD_STOP: {
        if (snap_.state == S::STATE_PAUSED) {
          if (snap_.controller_mode != CS::MODE_NORMAL &&
              snap_.controller_mode != CS::MODE_ESTOP)
          {
            // Defensive guard: if controller is somehow outside MODE_NORMAL while PAUSED,
            // transition through STATE_STOPPING so the worker returns it to MODE_NORMAL.
            ++session_id_;
            stop_requested_ = true;
            pause_requested_ = false;
            pause_deadline_ = Clock::time_point{};
            step_mode_ = false;
            snap_.state = S::STATE_STOPPING;
            phase_deadline_ = now + kStoppingTimeout;
            res.effects.cancel_controller_goals = true;
            res.effects.wake_worker = true;
            notify_unlocked("Stopping from PAUSED; returning controller to MODE_NORMAL");
            res.accepted = true;
            changed = true;
            break;
          }
          ++session_id_;
          stop_requested_ = false;
          pause_requested_ = false;
          pause_deadline_ = Clock::time_point{};
          step_mode_ = false;
          snap_.current_block_index = 0;
          snap_.current_line_number = 0;
          snap_.current_statement_index = -1;
          snap_.block_progress_percent = 0.0;
          snap_.total_progress_percent = 0.0;
          snap_.cartesian_pos_error_m.fill(0.0);
          snap_.cartesian_rot_error_rad.fill(0.0);
          snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
          notify_unlocked("Stopped from PAUSED; cursor reset to start");
          res.accepted = true;
          changed = true;
          break;
        }
        if (snap_.state != S::STATE_RUNNING && snap_.state != S::STATE_STEPPING) {
          reject("STOP is only valid from RUNNING, STEPPING, or PAUSED");
          break;
        }
        stop_requested_ = true;
        pause_requested_ = false;
        pause_deadline_ = Clock::time_point{};
        snap_.state = S::STATE_STOPPING;
        phase_deadline_ = now + kStoppingTimeout;
        res.effects.cancel_controller_goals = true;
        res.effects.wake_worker = true;
        notify_unlocked("Stopping execution cleanly");
        res.accepted = true;
        changed = true;
        break;
      }

      case C::Request::CMD_RECOVER: {
        if (snap_.state != S::STATE_ESTOP) {
          reject("RECOVER is only valid in STATE_ESTOP");
          break;
        }
        if (!j_fresh || !c_fresh) {
          reject(
            !c_fresh && controller_stamp_ != Clock::time_point{} ?
            std::string("RECOVER requires fresh /ctc_controller/status.\n") + kControllerLostHint :
            std::string("RECOVER requires fresh /joint_states and /ctc_controller/status"));
          break;
        }
        if (snap_.controller_mode != CS::MODE_ESTOP &&
            snap_.controller_mode != CS::MODE_NORMAL)
        {
          reject("RECOVER requires controller in MODE_ESTOP (or already MODE_NORMAL)");
          break;
        }
        if (snap_.controller_mode == CS::MODE_NORMAL &&
            last_estop_cmd_sent_ != Clock::time_point{} &&
            (now - last_estop_cmd_sent_) < kEstopCmdInFlightGuard)
        {
          reject("E-STOP latch command still in flight; wait briefly and retry RECOVER");
          break;
        }
        ++session_id_;
        stop_requested_ = false;
        pause_requested_ = false;
        pause_deadline_ = Clock::time_point{};
        step_mode_ = false;
        snap_.state = S::STATE_RECOVERING;
        recovery_phase_ = RecoverySubPhase::WAIT_SETTLE;
        recovery_initial_mode_ = snap_.controller_mode;
        recovery_initial_mode_epoch_ = last_mode_epoch_;
        phase_deadline_ = now + kRecoveryPhaseTimeout;
        reset_settle_unlocked();
        notify_unlocked("Waiting for stationary settle before E-STOP recovery");
        res.accepted = true;
        res.effects.wake_worker = true;
        changed = true;
        break;
      }

      case C::Request::CMD_RESET_FAULT: {
        if (snap_.state != S::STATE_FAULT) {
          reject("RESET_FAULT is only valid in STATE_FAULT");
          break;
        }
        if (!j_fresh || !c_fresh) {
          reject("RESET_FAULT requires fresh telemetry");
          break;
        }
        if (snap_.controller_mode == CS::MODE_ESTOP || snap_.estop_latched) {
          reject("Controller is in E-STOP; issue CMD_RECOVER instead");
          break;
        }
        if (snap_.controller_mode != CS::MODE_NORMAL) {
          // Non-blocking: request NORMAL and clear the fault once NORMAL is observed.
          res.effects.send_enter_normal_cmd = true;
          res.effects.controller_epoch = snap_.controller_epoch;
          fault_reset_pending_ = true;
          fault_reset_deadline_ = now + kFaultResetTimeout;
          notify_unlocked(
            "Returning controller to MODE_NORMAL; fault clears automatically once NORMAL is observed");
          res.accepted = true;
          changed = true;
          break;
        }
        clear_fault_unlocked("Fault cleared");
        res.accepted = true;
        changed = true;
        break;
      }

      case C::Request::CMD_CLEAR_PROGRAM: {
        if (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING ||
            snap_.state == S::STATE_STOPPING || snap_.state == S::STATE_RECOVERING)
        {
          reject("Cannot clear program while robot is active");
          break;
        }
        if (snap_.state == S::STATE_PAUSED &&
            snap_.controller_mode != CS::MODE_NORMAL &&
            snap_.controller_mode != CS::MODE_ESTOP)
        {
          reject("Controller is not in MODE_NORMAL; issue STOP or RESET_FAULT before clearing program");
          break;
        }
        has_program_ = false;
        snap_.loaded_program_id.clear();
        snap_.program_revision = 0;
        snap_.total_blocks = 0;
        snap_.current_block_index = 0;
        snap_.current_line_number = 0;
        snap_.current_statement_index = -1;
        snap_.block_progress_percent = 0.0;
        snap_.total_progress_percent = 0.0;
        if (snap_.state == S::STATE_COMPILED_READY || snap_.state == S::STATE_PAUSED) {
          snap_.state = S::STATE_IDLE;
        }
        notify_unlocked("Compiled program cleared");
        res.accepted = true;
        changed = true;
        break;
      }

      default:
        reject("Unknown StudioCommand value");
        break;
    }

    if (res.accepted) {
      res.planner_epoch = planner_epoch_;
      res.revision = snap_.revision;
      res.state = snap_.state;
      res.controller_epoch = snap_.controller_epoch;
      res.message = snap_.detail;
    }
    if (changed) {
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
  return res;
}

uint64_t StudioSupervisor::active_session() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_id_;
}

uint8_t StudioSupervisor::current_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snap_.state;
}

std::pair<uint8_t, uint64_t> StudioSupervisor::state_and_session() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {snap_.state, session_id_};
}

StudioSupervisor::RecoverySubPhase StudioSupervisor::recovery_subphase() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return recovery_phase_;
}

bool StudioSupervisor::stop_requested(uint64_t session) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session != session_id_ || stop_requested_;
}

bool StudioSupervisor::pause_requested(uint64_t session) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session == session_id_ && pause_requested_;
}

bool StudioSupervisor::can_dispatch(uint64_t session, bool honor_pause) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session != session_id_ || stop_requested_) {
    return false;
  }
  if (honor_pause && pause_requested_) {
    return false;
  }
  return true;
}

bool StudioSupervisor::is_step_mode(uint64_t session) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session == session_id_ && step_mode_;
}

bool StudioSupervisor::is_fresh_joint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fresh_joint_unlocked(Clock::now());
}

void StudioSupervisor::update_execution_progress(
  uint64_t session,
  int32_t line_number,
  int32_t statement_index,
  int32_t block_index,
  double block_progress_pct,
  double total_progress_pct)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session != session_id_) {
    return;
  }
  snap_.current_line_number = line_number;
  snap_.current_statement_index = statement_index;
  snap_.current_block_index = block_index;
  snap_.block_progress_percent = block_progress_pct;
  snap_.total_progress_percent = total_progress_pct;
}

void StudioSupervisor::update_impedance_errors(
  uint64_t session,
  const std::array<double, 3> & pos_err_m,
  const std::array<double, 3> & rot_err_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session != session_id_) {
    return;
  }
  snap_.cartesian_pos_error_m = pos_err_m;
  snap_.cartesian_rot_error_rad = rot_err_rad;
}

void StudioSupervisor::clear_impedance_errors()
{
  std::lock_guard<std::mutex> lock(mutex_);
  snap_.cartesian_pos_error_m.fill(0.0);
  snap_.cartesian_rot_error_rad.fill(0.0);
}

void StudioSupervisor::on_pause_settled(
  uint64_t session,
  int32_t line_number,
  int32_t statement_index,
  int32_t block_index,
  double block_progress_pct,
  double total_progress_pct)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_) {
      return;
    }
    if (snap_.state == S::STATE_STOPPING || stop_requested_) {
      finish_stop_unlocked("Program execution stopped cleanly");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    } else if (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING) {
      pause_requested_ = false;
      pause_deadline_ = Clock::time_point{};
      step_mode_ = false;
      snap_.state = S::STATE_PAUSED;
      snap_.current_line_number = line_number;
      snap_.current_statement_index = statement_index;
      snap_.current_block_index = block_index;
      snap_.block_progress_percent = block_progress_pct;
      snap_.total_progress_percent = total_progress_pct;
      notify_unlocked("Paused at line " + std::to_string(line_number));
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_step_block_done(
  uint64_t session,
  bool more_blocks_remaining,
  int32_t next_block_idx,
  int32_t next_line_number,
  int32_t next_statement_index,
  double total_progress_pct)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_) {
      return;
    }
    if (snap_.state == S::STATE_STOPPING || stop_requested_) {
      finish_stop_unlocked("Program execution stopped cleanly");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    } else if (snap_.state == S::STATE_STEPPING) {
      step_mode_ = false;
      pause_requested_ = false;
      pause_deadline_ = Clock::time_point{};
      snap_.cartesian_pos_error_m.fill(0.0);
      snap_.cartesian_rot_error_rad.fill(0.0);
      if (more_blocks_remaining) {
        snap_.state = S::STATE_PAUSED;
        snap_.current_block_index = next_block_idx;
        snap_.current_line_number = next_line_number;
        snap_.current_statement_index = next_statement_index;
        snap_.block_progress_percent = 0.0;
        snap_.total_progress_percent = total_progress_pct;
        notify_unlocked("Step complete; paused before block " + std::to_string(next_block_idx + 1));
      } else {
        snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
        snap_.current_block_index = snap_.total_blocks;
        snap_.block_progress_percent = 100.0;
        snap_.total_progress_percent = 100.0;
        notify_unlocked("Step reached end of program");
      }
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_program_completed(uint64_t session)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_) {
      return;
    }
    if (snap_.state == S::STATE_STOPPING || stop_requested_) {
      finish_stop_unlocked("Program execution stopped cleanly");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    } else if (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING) {
      pause_requested_ = false;
      pause_deadline_ = Clock::time_point{};
      step_mode_ = false;
      snap_.state = has_program_ ? S::STATE_COMPILED_READY : S::STATE_IDLE;
      snap_.current_block_index = snap_.total_blocks;
      snap_.block_progress_percent = 100.0;
      snap_.total_progress_percent = 100.0;
      snap_.cartesian_pos_error_m.fill(0.0);
      snap_.cartesian_rot_error_rad.fill(0.0);
      notify_unlocked("Program execution completed");
      changed = true;
      notify_snap = make_snapshot_unlocked(Clock::now());
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_stop_completed(uint64_t session)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_ || snap_.state != S::STATE_STOPPING) {
      return;
    }
    finish_stop_unlocked("Program execution stopped cleanly");
    changed = true;
    notify_snap = make_snapshot_unlocked(Clock::now());
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_execution_fault(
  uint64_t session,
  int32_t error_code,
  const std::string & detail,
  bool trigger_estop,
  Effects & out_effects)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_) {
      return;
    }
    const auto now = Clock::now();
    if (trigger_estop) {
      out_effects.send_estop_cmd = true;
      last_estop_cmd_sent_ = now;
      enter_estop_unlocked(detail, snap_.estop_reason, out_effects);
      changed = true;
    } else if (snap_.state == S::STATE_STOPPING || stop_requested_) {
      if (error_code == -38) {
        // Returning the controller to MODE_NORMAL failed during a controlled stop.
        out_effects.cancel_controller_goals = true;
        enter_fault_unlocked(error_code, detail);
        changed = true;
      } else if (snap_.controller_mode == CS::MODE_NORMAL) {
        // A transient block error (gripper/payload/replan) raced with an accepted STOP;
        // do not hijack STATE_STOPPING into STATE_FAULT once the controller is in MODE_NORMAL.
        finish_stop_unlocked("Program execution stopped cleanly");
        changed = true;
      } else {
        // Stay in STATE_STOPPING and wake the worker to finish returning to MODE_NORMAL.
        out_effects.cancel_controller_goals = true;
        out_effects.wake_worker = true;
      }
    } else {
      out_effects.cancel_controller_goals = true;
      enter_fault_unlocked(error_code, detail);
      changed = true;
    }
    if (changed) {
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::note_pause_deferred(const std::string & detail, double min_remaining_sec)
{
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pause_requested_ &&
        (snap_.state == S::STATE_RUNNING || snap_.state == S::STATE_STEPPING))
    {
      const auto now = Clock::now();
      if (min_remaining_sec > 0.0) {
        const auto extended = now + std::chrono::milliseconds(
          static_cast<int64_t>((min_remaining_sec + 3.0) * 1000.0));
        if (extended > pause_deadline_) {
          pause_deadline_ = extended;
        }
      }
      notify_unlocked(detail);
      changed = true;
      notify_snap = make_snapshot_unlocked(now);
    }
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

void StudioSupervisor::on_recovery_failed(uint64_t session, const std::string & reason)
{
  Effects dummy;
  bool changed = false;
  Snapshot notify_snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session != session_id_ || snap_.state != S::STATE_RECOVERING) {
      return;
    }
    enter_estop_unlocked("Recovery failed: " + reason, snap_.estop_reason, dummy);
    changed = true;
    notify_snap = make_snapshot_unlocked(Clock::now());
  }
  if (changed && on_change_) {
    on_change_(notify_snap);
  }
}

}  // namespace lite6_studio_planner
