#ifndef LITE6_PLANNER_SUPERVISOR_HPP_
#define LITE6_PLANNER_SUPERVISOR_HPP_

#include <lite6_interfaces/msg/compliance_settings.hpp>
#include <lite6_interfaces/msg/supervisor_state.hpp>
#include <lite6_interfaces/msg/controller_status.hpp>
#include <lite6_interfaces/srv/controller_command.hpp>
#include <lite6_interfaces/srv/supervisor_command.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace lite6_planner {

// Sole authoritative DES/SCT state machine. All writes are serialized here.
// GUI only issues events and consumes snapshots. CTC mode remains an observation.
class Supervisor {
public:
  using State=lite6_interfaces::msg::SupervisorState;
  using Command=lite6_interfaces::srv::SupervisorCommand;
  struct Effects {
    uint8_t ctc_cmd = lite6_interfaces::srv::ControllerCommand::Request::CMD_NONE;
    bool has_compliance = false;
    lite6_interfaces::msg::ComplianceSettings compliance{};
    bool has_payload = false;
    double payload_mass = 0.0;
    std::array<double, 3> payload_com_tcp{0.0, 0.0, 0.0};
    uint64_t expected_controller_epoch = 0;
    std::string cmd_reason;
    bool cancel_active = false;
    bool terminal_success = false;
    std::string cancel_key;
  };
  struct Reply {
    bool accepted = false;
    uint64_t planner_epoch = 0;
    uint64_t session_id = 0;
    uint64_t revision = 0;
    uint8_t mode = 0;
    std::string message;
    Effects effects;
  };

  Supervisor();
  uint64_t planner_epoch() const { return planner_epoch_; }
  void observe_joint_state(double max_speed, int64_t sample_steady_ns);
  void observe_excitation_start(const std::string& key, const builtin_interfaces::msg::Time& start);
  Effects observe_controller(const lite6_interfaces::msg::ControllerStatus& status);
  // outcome_unknown=true (or ack_result==ACK_OUTCOME_UNKNOWN): the command may or may
  // not have been applied; reconcile from observed ControllerStatus telemetry.
  void on_controller_command_ack(uint8_t cmd, bool accepted, uint8_t ack_result,
    uint8_t current_mode, const std::string& msg, bool outcome_unknown=false,
    uint64_t cmd_seq=0);
  Effects watchdog();
  State snapshot() const;
  Reply command(const Command::Request& req);

  // Key is an opaque ROS action UUID (hex) or a unique SysID excitation token.
  bool reserve_motion(const std::string& key,uint8_t context,uint64_t session,std::string& why);
  bool reserve_excitation(uint64_t session,std::string& key,std::string& why);
  bool begin_excitation_validation(uint64_t session,std::string& why);
  void end_excitation_validation(uint64_t session);
  bool permitted(const std::string& key) const;
  bool change_motion_phase(const std::string& key,uint8_t phase);
  bool request_cancel(const std::string& key);
  Effects finish_motion(const std::string& key,bool succeeded,int32_t code,const std::string& message);
  std::string active_motion_key() const;
  bool controller_fresh() const;

private:
  using Clock=std::chrono::steady_clock;
  void changed_unlocked(const std::string& detail="");
  bool fresh_controller_unlocked() const;
  bool fresh_joint_unlocked() const;
  bool can_start_motion_unlocked(uint8_t context,uint64_t session,std::string& reason) const;
  void fault_unlocked(const std::string& reason,bool estop,Effects& effects,
    uint8_t estop_reason=State::ESTOP_NONE);
  void clear_motion_unlocked();
  void settle_unlocked();
  void complete_sysid_unlocked();
  bool sync_from_initializing_unlocked();
  mutable std::mutex mutex_;
  uint64_t planner_epoch_=0;
  State state_;
  Clock::time_point controller_time_{},phase_deadline_{};
  int64_t joint_sample_ns_=0;
  int64_t stable_since_ns_=0;
  std::string sysid_failure_;
  uint8_t compliance_target_=0;
  uint8_t pre_switch_mode_=State::MODE_NORMAL;
  // Set when a compliance switch command's outcome is unknown (client timeout or ACK_OUTCOME_UNKNOWN).
  Clock::time_point switch_unknown_deadline_{};
  // Telemetry reconciliation state for CMD_SET_PAYLOAD with unknown outcome.
  bool payload_unknown_pending_=false;
  uint64_t payload_unknown_seq_=0;
  uint64_t payload_pre_seq_=0;
  uint64_t last_cmd_seq_=0;
  // Latched once per controller-status outage to show the reactivation hint.
  bool controller_lost_notified_=false;
  uint64_t last_controller_epoch_=0;
  bool seen_controller_=false,seen_joint_=false;
  std::string active_key_;
  bool cancel_requested_=false;
  uint64_t next_session_=1;
};

} // namespace lite6_planner
#endif
