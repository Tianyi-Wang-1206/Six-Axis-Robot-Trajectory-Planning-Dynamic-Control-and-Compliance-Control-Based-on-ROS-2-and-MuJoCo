#ifndef LITE6_PLANNER_SUPERVISOR_HPP_
#define LITE6_PLANNER_SUPERVISOR_HPP_

#include <lite6_interfaces/msg/supervisor_state.hpp>
#include <lite6_interfaces/msg/controller_status.hpp>
#include <lite6_interfaces/srv/supervisor_command.hpp>
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
  struct Effects { int ctc_command=-1; bool cancel_active=false; bool terminal_success=false; std::string cancel_key; };
  struct Reply {bool accepted=false; uint64_t session_id=0; uint64_t revision=0;std::string message; Effects effects;};

  Supervisor();
  void observe_joint_state(double max_speed);
  void observe_excitation_start(const std::string& key, const builtin_interfaces::msg::Time& start);
  Effects observe_controller(uint8_t mode);
  Effects watchdog();
  State snapshot() const;
  Reply command(uint8_t event,uint64_t session,const std::string& reason);

  // Key is an opaque ROS action UUID (hex) or a unique SysID excitation token.
  bool reserve_motion(const std::string& key,uint8_t context,uint64_t session,std::string& why);
  bool reserve_excitation(uint64_t session,std::string& key,std::string& why);
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
  void fault_unlocked(const std::string& reason,bool estop,Effects& effects);
  void clear_motion_unlocked();
  void settle_unlocked();
  void complete_sysid_unlocked();
  mutable std::mutex mutex_;
  State state_;
  Clock::time_point controller_time_{},joint_time_{},phase_deadline_{};
  Clock::time_point stable_since_{};
  std::string sysid_failure_;
  bool seen_controller_=false,seen_joint_=false;
  std::string active_key_;
  bool cancel_requested_=false;
  uint64_t next_session_=1;
};

} // namespace lite6_planner
#endif
