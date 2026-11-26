#include "lite6_planner/supervisor.hpp"
#include <lite6_interfaces/compliance_validation.hpp>
#include <lite6_interfaces/motion_contract.hpp>
#include <lite6_interfaces/shm_registers.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>

namespace lite6_planner {
namespace {
using S=lite6_interfaces::msg::SupervisorState;
using C=lite6_interfaces::srv::SupervisorCommand;
using CS=lite6_interfaces::msg::ControllerStatus;
using CTC=lite6_interfaces::srv::ControllerCommand;
constexpr auto kControllerObservationTimeout=
  std::chrono::nanoseconds(lite6_interfaces::contract::kControllerStatusStaleTimeoutNs);
constexpr auto kModeTimeout=std::chrono::seconds(8);
constexpr int64_t kSettleDurationNs=300'000'000LL;
constexpr int64_t kSettleFeedbackGapNs=100'000'000LL;
constexpr auto kDispatchTimeout=std::chrono::seconds(15);
constexpr auto kValidationTimeout=std::chrono::seconds(120);
constexpr auto kCalculationTimeout=std::chrono::seconds(60);
constexpr double kSettleSpeed=lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec;
constexpr double kRadToDeg=57.29577951308232;
inline int64_t steady_ns(std::chrono::steady_clock::time_point tp) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}
// Grace period before a compliance switch with an unknown CTC outcome is resolved
// from the observed controller mode (the CTC answers or withdraws within ~0.15 s).
constexpr auto kSwitchUnknownGrace=std::chrono::milliseconds(1500);
// An RT fault (fail_controller) makes controller_manager deactivate the CTC; it then
// stops publishing status and no supervisor command can bring it back.
const char* const kControllerLostHint=
  "No /ctc_controller/status heartbeat: ctc_controller may have been deactivated by "
  "controller_manager after an RT fault (non-finite feedback, effort write failure or "
  "update exception). Reactivate it with: ros2 service call "
  "/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "
  "\"{activate_controllers: [ctc_controller], strictness: 2}\" -- then recover the E-STOP.";

uint8_t map_compliance_supervisor_mode(uint8_t ctc_mode) {
  switch (ctc_mode) {
    case CS::MODE_NORMAL: return S::MODE_NORMAL;
    case CS::MODE_JOINT_IMPEDANCE: return S::MODE_JOINT_IMPEDANCE;
    case CS::MODE_CARTESIAN_IMPEDANCE: return S::MODE_CARTESIAN_IMPEDANCE;
    case CS::MODE_TEACH: return S::MODE_TEACH;
    default: return S::MODE_INITIALIZING;
  }
}

uint8_t map_compliance_controller_mode(uint8_t sup_mode) {
  switch (sup_mode) {
    case S::MODE_NORMAL: return CS::MODE_NORMAL;
    case S::MODE_JOINT_IMPEDANCE: return CS::MODE_JOINT_IMPEDANCE;
    case S::MODE_CARTESIAN_IMPEDANCE: return CS::MODE_CARTESIAN_IMPEDANCE;
    case S::MODE_TEACH: return CS::MODE_TEACH;
    default: return CS::MODE_UNKNOWN;
  }
}

std::string format_fixed(double value,int precision=3) {
  std::ostringstream out;
  out<<std::fixed<<std::setprecision(precision)<<value;
  return out.str();
}

std::string joint_label(int8_t joint) {
  return (joint>=0&&joint<6)?("joint"+std::to_string(joint+1)):"arm joint";
}

std::string mode_label(uint8_t mode) {
  switch(mode) {
    case S::MODE_JOINT_IMPEDANCE: return "Joint-space impedance";
    case S::MODE_CARTESIAN_IMPEDANCE: return "Cartesian-space impedance";
    case S::MODE_TEACH: return "Drag teaching";
    case S::MODE_COMPLIANCE_SWITCHING: return "Compliance mode transition";
    case S::MODE_SYSID: return "Friction Identification";
    case S::MODE_NORMAL: return "NORMAL";
    default: return "Active control mode";
  }
}

uint8_t map_supervisor_estop_reason(uint8_t ctc_reason) {
  switch(ctc_reason) {
    case CS::ESTOP_COMPLIANCE_POSITION_LIMIT: return S::ESTOP_COMPLIANCE_POSITION_LIMIT;
    case CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT: return S::ESTOP_COMPLIANCE_VELOCITY_LIMIT;
    case CS::ESTOP_COMPLIANCE_SATURATION: return S::ESTOP_COMPLIANCE_SATURATION;
    case CS::ESTOP_TRAJECTORY_PATH_POSITION:
    case CS::ESTOP_TRAJECTORY_PATH_VELOCITY:
    case CS::ESTOP_TRAJECTORY_GOAL_TIMEOUT:
    case CS::ESTOP_TRAJECTORY_SATURATION:
    case CS::ESTOP_TRAJECTORY_MODE_CHANGED: return S::ESTOP_TRAJECTORY;
    default: return S::ESTOP_CONTROLLER_FAULT;
  }
}

std::string describe_controller_estop(const CS& status,uint8_t previous_mode) {
  const std::string joint=joint_label(status.estop_joint);
  const std::string mode_name=mode_label(previous_mode);
  switch(status.estop_reason) {
    case CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT: {
      const double speed=std::abs(status.estop_value);
      return "Compliance E-STOP: "+joint+" velocity limit exceeded ("+
        format_fixed(speed,2)+" rad/s > "+format_fixed(status.estop_limit,2)+" rad/s)\n"+
        "Active mode: "+mode_name+"\n"+
        "Cause: "+joint+" speed reached "+format_fixed(status.estop_value,3)+" rad/s ("+
        format_fixed(status.estop_value*kRadToDeg,1)+" deg/s), exceeding the compliance safety limit of "+
        format_fixed(status.estop_limit,2)+" rad/s ("+format_fixed(status.estop_limit*kRadToDeg,1)+" deg/s).\n"+
        "Improper operation: The applied MuJoCo mouse drag force or torque accelerated the arm too abruptly. "
        "Drag more gently and smoothly, then press RESET FAULT to recover.";
    }
    case CS::ESTOP_COMPLIANCE_POSITION_LIMIT:
      return "Compliance E-STOP: "+joint+" position limit exceeded ("+
        format_fixed(status.estop_value,3)+" rad vs limit "+format_fixed(status.estop_limit,3)+" rad)\n"+
        "Active mode: "+mode_name+"\n"+
        "Cause: "+joint+" reached "+format_fixed(status.estop_value,3)+" rad ("+
        format_fixed(status.estop_value*kRadToDeg,1)+" deg), exceeding its mechanical travel limit of "+
        format_fixed(status.estop_limit,3)+" rad ("+format_fixed(status.estop_limit*kRadToDeg,1)+" deg).\n"+
        "Improper operation: The applied MuJoCo mouse drag force pushed the joint beyond its physical boundary. "
        "Avoid pulling joints hard against their travel limits, then press RESET FAULT to recover.";
    case CS::ESTOP_COMPLIANCE_SATURATION:
      return "Compliance E-STOP: "+joint+" sustained torque saturation ("+
        format_fixed(status.estop_value,2)+" s > "+format_fixed(status.estop_limit,2)+" s)\n"+
        "Active mode: "+mode_name+"\n"+
        "Cause: "+joint+" commanded motor torque stayed saturated for "+
        format_fixed(status.estop_value,2)+" s, exceeding the "+format_fixed(status.estop_limit,2)+" s timeout.\n"+
        "Improper operation: The applied MuJoCo mouse drag force pulled too strongly against the compliance "
        "resistance or joint-limit wall for too long. Reduce the drag force or displacement (or lower stiffness), "
        "then press RESET FAULT to recover.";
    case CS::ESTOP_COMPLIANCE_STOPPING_TIMEOUT:
      return "Compliance E-STOP: transition braking timed out ("+
        format_fixed(status.estop_value,2)+" s >= "+format_fixed(status.estop_limit,2)+" s)\n"+
        "Active mode: "+mode_name+"\n"+
        "Cause: External load or motion prevented safe settling during compliance exit.\n"
        "Release external load, then press RESET FAULT to recover.";
    case CS::ESTOP_TRAJECTORY_PATH_POSITION:
      return "CTC E-STOP: "+joint+" path position error exceeded ("+
        format_fixed(status.estop_value,3)+" rad > "+format_fixed(status.estop_limit,3)+" rad)\n"+
        "Active mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_TRAJECTORY_PATH_VELOCITY:
      return "CTC E-STOP: "+joint+" path velocity error exceeded ("+
        format_fixed(status.estop_value,3)+" rad/s > "+format_fixed(status.estop_limit,3)+" rad/s)\n"+
        "Active mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_TRAJECTORY_SATURATION:
      return "CTC E-STOP: "+joint+" continuous effort saturation ("+
        format_fixed(status.estop_value,2)+" s >= "+format_fixed(status.estop_limit,2)+" s)\n"+
        "Active mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_TRAJECTORY_GOAL_TIMEOUT:
      return "CTC E-STOP: goal settling deadline exceeded\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_TRAJECTORY_MODE_CHANGED:
      return "CTC E-STOP: controller mode changed during trajectory execution\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_INVALID_STATE:
      return "CTC E-STOP: non-finite joint state feedback\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_INVALID_COMMAND:
      return "CTC E-STOP: non-finite torque command\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_COMMAND_WRITE:
      return "CTC E-STOP: hardware effort command write failed\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    case CS::ESTOP_CONTROL_EXCEPTION:
      return "CTC E-STOP: exception in real-time control update\nActive mode: "+mode_name+"\nPress RESET FAULT to recover.";
    default:
      return "CTC reported E-STOP\nActive mode: "+mode_name+
        "\nCause: Controller entered emergency stop externally or without a specific safety code.\n"
        "Press RESET FAULT to recover.";
  }
}
}

Supervisor::Supervisor() {
  // Session identifiers are correlation tokens, not authentication secrets.
  std::random_device rd;
  next_session_=(static_cast<uint64_t>(rd())<<32) | static_cast<uint64_t>(rd());
  if(next_session_==0) next_session_=1;
  const int64_t boot_ns=lite6_interfaces::shm::steady_now_ns();
  planner_epoch_=static_cast<uint64_t>(boot_ns>0?boot_ns:1);
  state_.planner_epoch=planner_epoch_;
  state_.mode=S::MODE_INITIALIZING;
  state_.motion_phase=S::MOTION_IDLE;
  state_.sysid_phase=S::SYSID_NONE;
  state_.recovery_phase=S::RECOVERY_NONE;
  state_.controller_mode=CS::MODE_UNKNOWN;
  state_.revision=1;
  state_.session_id=0;
  state_.detail="Waiting for CTC mode and joint states";
}
void Supervisor::changed_unlocked(const std::string& description) {
  ++state_.revision;
  if(!description.empty()) state_.detail=description;
}
bool Supervisor::fresh_controller_unlocked() const {
  return seen_controller_&&(Clock::now()-controller_time_<kControllerObservationTimeout);
}
bool Supervisor::fresh_joint_unlocked() const {
  return seen_joint_&&lite6_interfaces::shm::is_stamp_fresh(
    joint_sample_ns_,lite6_interfaces::shm::steady_now_ns(),
    lite6_interfaces::contract::kJointStateStaleTimeoutNs);
}
bool Supervisor::sync_from_initializing_unlocked() {
  if(state_.mode!=S::MODE_INITIALIZING||!fresh_controller_unlocked()||!fresh_joint_unlocked())
    return false;
  const uint8_t mapped=map_compliance_supervisor_mode(state_.controller_mode);
  if(mapped==S::MODE_INITIALIZING) return false;
  state_.mode=mapped;
  changed_unlocked("Ready");
  return true;
}
void Supervisor::observe_joint_state(double max_speed,int64_t sample_steady_ns) {
  std::lock_guard<std::mutex> guard(mutex_);
  const int64_t now_ns=lite6_interfaces::shm::steady_now_ns();
  // Preserve the source steady-clock timestamp and reject stale, future, or older samples
  // so joint freshness can never extend beyond the 100 ms contract.
  if(!lite6_interfaces::shm::is_stamp_fresh(
        sample_steady_ns,now_ns,lite6_interfaces::contract::kJointStateStaleTimeoutNs)||
     (seen_joint_&&sample_steady_ns<joint_sample_ns_)) {
    return;
  }
  if(!seen_joint_||
     !lite6_interfaces::shm::is_stamp_fresh(
        joint_sample_ns_,sample_steady_ns,kSettleFeedbackGapNs)||
     !std::isfinite(max_speed)||max_speed>kSettleSpeed) {
    stable_since_ns_=0;
  }
  if(std::isfinite(max_speed)&&max_speed<=kSettleSpeed&&stable_since_ns_==0) {
    stable_since_ns_=sample_steady_ns;
  }
  seen_joint_=true;
  joint_sample_ns_=sample_steady_ns;
  sync_from_initializing_unlocked();
}
void Supervisor::observe_excitation_start(const std::string& key,
    const builtin_interfaces::msg::Time& start) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(active_key_!=key||state_.mode!=S::MODE_SYSID||state_.sysid_phase!=S::SYSID_EXCITING||
      start.sec<=0||state_.excitation_start.sec!=0) return;
  state_.excitation_start=start;
  changed_unlocked("CTC excitation clock synchronized");
}
void Supervisor::settle_unlocked() {
  state_.waiting_for_settle=true;
  stable_since_ns_=0;
  phase_deadline_=Clock::now()+kModeTimeout;
}
void Supervisor::complete_sysid_unlocked() {
  state_.mode=S::MODE_NORMAL;
  state_.sysid_phase=S::SYSID_NONE;
  state_.session_id=0;
  state_.waiting_for_settle=false;
  phase_deadline_=Clock::time_point{};
  state_.last_error_code=sysid_failure_.empty()?1:S::ERROR_SYSID_ESTIMATION;
  changed_unlocked(sysid_failure_.empty()?"Friction identification completed; NORMAL and stationary":
      "Friction identification failed; NORMAL restored: "+sysid_failure_);
}
void Supervisor::clear_motion_unlocked() {
  active_key_.clear();cancel_requested_=false;
  state_.motion_phase=S::MOTION_IDLE;
}
void Supervisor::fault_unlocked(const std::string& reason,bool estop,Effects& effect,
    uint8_t estop_reason) {
  if(!active_key_.empty()) {
    effect.cancel_active=true;effect.cancel_key=active_key_;
    cancel_requested_=true;
    state_.motion_phase=S::MOTION_CANCELING;
  }
  switch_unknown_deadline_=Clock::time_point{};
  if(estop) {
    state_.mode=S::MODE_ESTOP;
    state_.estop_latched=true;
    state_.estop_reason=estop_reason!=S::ESTOP_NONE?estop_reason:S::ESTOP_SUPERVISOR;
    effect.ctc_cmd=CTC::Request::CMD_ESTOP;
    effect.expected_controller_epoch=0;
    effect.cmd_reason=reason;
  } else state_.mode=S::MODE_SOFT_FAULT;
  state_.sysid_phase=S::SYSID_NONE;
  state_.recovery_phase=S::RECOVERY_NONE;
  state_.session_id=0;
  state_.waiting_for_settle=false;
  phase_deadline_=Clock::time_point{};
  state_.last_error_code=-7;
  changed_unlocked(reason);
}
Supervisor::Effects Supervisor::observe_controller(const CS& status) {
  std::lock_guard<std::mutex> guard(mutex_);
  Effects fx;
  const uint8_t mode=status.mode;
  if(mode<CS::MODE_NORMAL||mode>CS::MODE_CARTESIAN_IMPEDANCE_TRAJECTORY) return fx;
  const auto previous=state_.controller_mode;
  const uint64_t prev_epoch=last_controller_epoch_;
  const bool was_lost_notified=controller_lost_notified_;
  seen_controller_=true;controller_time_=Clock::now();
  controller_lost_notified_=false;
  if(was_lost_notified) {
    const std::string suffix=std::string("\n\n")+kControllerLostHint;
    if(state_.detail.size()>=suffix.size()&&
       state_.detail.compare(state_.detail.size()-suffix.size(),suffix.size(),suffix)==0) {
      state_.detail.erase(state_.detail.size()-suffix.size());
    }
  }
  state_.controller_mode=mode;
  state_.controller_epoch=status.controller_epoch;
  state_.payload_mass=status.payload_mass;
  state_.payload_com_tcp=status.payload_com_tcp;
  last_cmd_seq_=status.last_cmd_seq;
  if(status.controller_epoch!=0) last_controller_epoch_=status.controller_epoch;

  if(payload_unknown_pending_&&
     ((payload_unknown_seq_>0&&status.last_cmd_seq>=payload_unknown_seq_)||
      (payload_unknown_seq_==0&&status.last_cmd_seq>payload_pre_seq_))) {
    if(status.last_cmd_code==CTC::Request::CMD_SET_PAYLOAD) {
      payload_unknown_pending_=false;
      if(status.last_cmd_result==CS::ACK_APPLIED) {
        changed_unlocked("Payload update confirmed via ControllerStatus telemetry");
      } else {
        changed_unlocked("Payload update rejected by CTC (reconciled via ControllerStatus, ack="+
          std::to_string(status.last_cmd_result)+")");
      }
    }
  }

  if(prev_epoch!=0&&status.controller_epoch!=0&&status.controller_epoch!=prev_epoch) {
    if(state_.mode==S::MODE_SYSID||state_.mode==S::MODE_RECOVERING||
       state_.mode==S::MODE_COMPLIANCE_SWITCHING||!active_key_.empty()) {
      fault_unlocked("Controller epoch changed mid-operation",mode==CS::MODE_ESTOP,fx);
      if(mode==CS::MODE_ESTOP) fx.ctc_cmd=CTC::Request::CMD_NONE;
      return fx;
    }
  }
  // Keep the initiating fault visible while the controller acknowledges ESTOP.
  if(state_.mode==S::MODE_ESTOP) {
    if(previous!=mode||was_lost_notified) changed_unlocked();
    return fx;
  }
  if(mode==CS::MODE_ESTOP&&state_.mode!=S::MODE_ESTOP&&state_.mode!=S::MODE_RECOVERING) {
    fault_unlocked(describe_controller_estop(status,state_.mode),true,fx,
      map_supervisor_estop_reason(status.estop_reason));
    fx.ctc_cmd=CTC::Request::CMD_NONE; // already in ESTOP; do not echo a redundant command
    return fx;
  }
  if(state_.mode==S::MODE_COMPLIANCE_SWITCHING && mode==compliance_target_) {
    state_.mode=map_compliance_supervisor_mode(mode);
    phase_deadline_=Clock::time_point{};
    switch_unknown_deadline_=Clock::time_point{};
    changed_unlocked("Compliance transition acknowledged by controller");
    return fx;
  }
  if(mode==CS::MODE_SYSID&&state_.mode!=S::MODE_SYSID) {
    fault_unlocked("CTC entered friction identification mode without supervisor authorization",true,fx,S::ESTOP_CONTROLLER_FAULT);
  } else if(state_.mode==S::MODE_INITIALIZING) {
    sync_from_initializing_unlocked();
  } else if(state_.mode==S::MODE_SYSID&&state_.sysid_phase==S::SYSID_WAIT_CONTROLLER_MODE&&
      !state_.waiting_for_settle&&mode==CS::MODE_SYSID) {
    state_.sysid_phase=S::SYSID_READY_FOR_EXCITATION;
    settle_unlocked();
    changed_unlocked("CTC confirmed friction identification mode; waiting for physical settling");
  } else if(state_.mode==S::MODE_SYSID&&state_.sysid_phase==S::SYSID_WAIT_NORMAL_MODE&&
      !state_.waiting_for_settle&&mode==CS::MODE_NORMAL) {
    state_.sysid_phase=S::SYSID_RETURN_ALIGN;
    settle_unlocked();
    changed_unlocked("CTC confirmed NORMAL; waiting for physical settling");
  } else if(state_.mode==S::MODE_RECOVERING&&
      state_.recovery_phase==S::RECOVERY_WAIT_NORMAL_MODE&&mode==CS::MODE_NORMAL&&
      !state_.waiting_for_settle&&fresh_joint_unlocked()&&active_key_.empty()) {
    state_.recovery_phase=S::RECOVERY_SETTLING;
    settle_unlocked();
    changed_unlocked("CTC confirmed NORMAL; waiting for recovery settling");
  } else if(previous!=mode) changed_unlocked("CTC mode observation changed");
  return fx;
}
void Supervisor::on_controller_command_ack(uint8_t cmd,bool accepted,uint8_t ack_result,
    uint8_t current_mode,const std::string& msg,bool outcome_unknown,uint64_t cmd_seq) {
  std::lock_guard<std::mutex> guard(mutex_);
  const bool unknown=outcome_unknown||ack_result==CTC::Response::ACK_OUTCOME_UNKNOWN;
  // SET_PAYLOAD does not change the supervisor mode; report the CTC's
  // actual outcome instead of announcing success when the event is issued.
  if(cmd==CTC::Request::CMD_SET_PAYLOAD) {
    const std::string what="Payload update";
    if(accepted) {
      payload_unknown_pending_=false;
      changed_unlocked("Payload configuration accepted by CTC");
    } else if(unknown) {
      payload_unknown_pending_=true;
      payload_unknown_seq_=cmd_seq;
      payload_pre_seq_=last_cmd_seq_;
      changed_unlocked(what+" outcome unknown; reconciling via ControllerStatus telemetry");
    } else {
      payload_unknown_pending_=false;
      changed_unlocked(what+" rejected by CTC ("+std::to_string(ack_result)+"): "+msg);
    }
    return;
  }
  if(cmd==CTC::Request::CMD_UPDATE_COMPLIANCE_SETTINGS) {
    if(accepted) {
      changed_unlocked("Live compliance settings updated");
    } else if(unknown) {
      changed_unlocked("Live compliance update outcome unknown; verify controller state");
    } else {
      changed_unlocked("Compliance update rejected by CTC ("+std::to_string(ack_result)+"): "+msg);
    }
    return;
  }
  if(accepted) return;
  if(unknown) {
    // Never assume rejection: the command may still have been applied. Compliance
    // switches resolve from the observed mode after a short grace period; recovery
    // and SysID sequences advance on observation or fail via phase_deadline_.
    if(state_.mode==S::MODE_COMPLIANCE_SWITCHING) {
      switch_unknown_deadline_=Clock::now()+kSwitchUnknownGrace;
      changed_unlocked("CTC compliance command outcome unknown; resolving from observed controller mode");
    }
    return;
  }
  if(state_.mode==S::MODE_COMPLIANCE_SWITCHING) {
    // ACK_RT_TIMEOUT is definitive here: the CTC withdrew the command (never applied).
    const uint8_t mapped=map_compliance_supervisor_mode(current_mode);
    state_.mode=(mapped!=S::MODE_INITIALIZING)?mapped:pre_switch_mode_;
    phase_deadline_=Clock::time_point{};
    switch_unknown_deadline_=Clock::time_point{};
    changed_unlocked("Compliance transition rejected by CTC ("+
      std::to_string(ack_result)+"): "+msg);
  } else if(state_.mode==S::MODE_RECOVERING&&cmd==CTC::Request::CMD_CLEAR_ESTOP&&
            !state_.waiting_for_settle) {
    if(ack_result==CTC::Response::ACK_REJECTED_SPEED) {
      settle_unlocked();
      changed_unlocked("Waiting for joint speed to drop below clear-ESTOP limit");
    } else {
      Effects unused;
      fault_unlocked("CTC rejected CMD_CLEAR_ESTOP: "+msg,true,unused,S::ESTOP_CONTROLLER_FAULT);
    }
  } else if(state_.mode==S::MODE_SYSID&&!state_.waiting_for_settle&&
            (cmd==CTC::Request::CMD_ENTER_SYSID||cmd==CTC::Request::CMD_ENTER_NORMAL)) {
    if(ack_result==CTC::Response::ACK_REJECTED_SPEED) {
      settle_unlocked();
      changed_unlocked("Waiting for joint speed to settle before friction identification mode change");
    } else {
      Effects unused;
      fault_unlocked("CTC rejected friction identification transition: "+msg,true,unused,S::ESTOP_CONTROLLER_FAULT);
    }
  }
}
Supervisor::Effects Supervisor::watchdog() {
  std::lock_guard<std::mutex> guard(mutex_);
  Effects fx;
  if(state_.mode==S::MODE_INITIALIZING) return fx;
  // MODE_FAULT (SupervisorState) is reserved by the frozen interface but never entered:
  // controller faults latch MODE_ESTOP and recoverable motion failures MODE_SOFT_FAULT.
  if((state_.mode==S::MODE_ESTOP||state_.mode==S::MODE_SOFT_FAULT)&&
     seen_controller_&&!fresh_controller_unlocked()&&!controller_lost_notified_) {
    controller_lost_notified_=true;
    changed_unlocked(state_.detail+"\n\n"+kControllerLostHint);
  }
  if(state_.mode==S::MODE_ESTOP) {
    if(fresh_controller_unlocked()&&state_.controller_mode!=CS::MODE_ESTOP) {
      fx.ctc_cmd=CTC::Request::CMD_ESTOP;
      fx.expected_controller_epoch=0;
      fx.cmd_reason="Re-latching controller E-STOP to match SupervisorState";
    }
    return fx;
  }
  if(state_.mode==S::MODE_SOFT_FAULT) return fx;
  if(!fresh_controller_unlocked()||!fresh_joint_unlocked()) {
    if(!fresh_controller_unlocked()) {
      controller_lost_notified_=true;
      fault_unlocked(std::string("CTC status heartbeat expired\n\n")+kControllerLostHint,true,fx);
    } else {
      fault_unlocked("joint_states heartbeat expired",true,fx);
    }
    return fx;
  }
  if(state_.mode==S::MODE_NORMAL&&state_.controller_mode!=CS::MODE_NORMAL) {
    fault_unlocked("Unexpected controller mode in NORMAL operation",true,fx);
    return fx;
  }
  if(state_.mode>=S::MODE_JOINT_IMPEDANCE&&state_.mode<=S::MODE_TEACH&&
     state_.controller_mode!=map_compliance_controller_mode(state_.mode)) {
    fault_unlocked("Unexpected controller mode in compliance operation",true,fx);
    return fx;
  }
  if(state_.mode==S::MODE_SYSID) {
    const auto phase=state_.sysid_phase;
    const bool requires_normal=(phase==S::SYSID_PRE_ALIGN||phase==S::SYSID_RETURN_ALIGN||
       phase==S::SYSID_COMPLETING||
       (phase==S::SYSID_WAIT_CONTROLLER_MODE&&state_.waiting_for_settle));
    const bool requires_sysid=(phase==S::SYSID_READY_FOR_EXCITATION||
       phase==S::SYSID_VALIDATING||
       phase==S::SYSID_EXCITING||phase==S::SYSID_CALCULATING||phase==S::SYSID_POST_ALIGN||
       (phase==S::SYSID_WAIT_NORMAL_MODE&&state_.waiting_for_settle));
    if((requires_normal&&state_.controller_mode!=CS::MODE_NORMAL)||
       (requires_sysid&&state_.controller_mode!=CS::MODE_SYSID)) {
      fault_unlocked("Unexpected CTC mode in friction identification phase",true,fx);
      return fx;
    }
  }
  if(state_.mode==S::MODE_RECOVERING&&
     state_.recovery_phase==S::RECOVERY_WAIT_NORMAL_MODE&&
     state_.controller_mode!=CS::MODE_ESTOP&&
     state_.controller_mode!=CS::MODE_NORMAL) {
    fault_unlocked("CTC mode mismatch while clearing ESTOP",true,fx);
    return fx;
  }
  if(state_.mode==S::MODE_RECOVERING&&state_.recovery_phase==S::RECOVERY_SETTLING&&
      state_.controller_mode!=CS::MODE_NORMAL) {
    fault_unlocked("CTC mode mismatch during recovery settling",true,fx);
    return fx;
  }
  const auto now=Clock::now();
  if(state_.mode==S::MODE_COMPLIANCE_SWITCHING&&
     switch_unknown_deadline_!=Clock::time_point{}&&now>switch_unknown_deadline_) {
    const uint8_t observed=map_compliance_supervisor_mode(state_.controller_mode);
    if(observed!=S::MODE_INITIALIZING&&state_.controller_mode!=compliance_target_) {
      state_.mode=observed;
      phase_deadline_=Clock::time_point{};
      switch_unknown_deadline_=Clock::time_point{};
      changed_unlocked("Compliance command was not applied (no CTC response); "
        "staying in the observed controller mode");
      return fx;
    }
  }
  if(phase_deadline_!=Clock::time_point{}&&now>phase_deadline_) {
    fault_unlocked(state_.waiting_for_settle?"Physical settling timed out":
        (state_.sysid_phase==S::SYSID_VALIDATING?"Friction identification trajectory validation timed out":
        "Sequence phase timed out (mode acknowledgement / client response)"),true,fx);
    return fx;
  }
  const int64_t now_ns=steady_ns(now);
  if(state_.waiting_for_settle&&stable_since_ns_>0&&
      lite6_interfaces::shm::is_stamp_fresh(joint_sample_ns_,now_ns,kSettleFeedbackGapNs)&&
      (joint_sample_ns_-stable_since_ns_)>=kSettleDurationNs) {
    state_.waiting_for_settle=false;
    phase_deadline_=now+kDispatchTimeout;
    if(state_.mode==S::MODE_SYSID) {
      if(state_.sysid_phase==S::SYSID_WAIT_CONTROLLER_MODE) {
        fx.ctc_cmd=CTC::Request::CMD_ENTER_SYSID;
        fx.expected_controller_epoch=state_.controller_epoch;
        fx.cmd_reason="Friction identification pre-alignment settled";
        phase_deadline_=now+kModeTimeout;
      } else if(state_.sysid_phase==S::SYSID_WAIT_NORMAL_MODE) {
        fx.ctc_cmd=CTC::Request::CMD_ENTER_NORMAL;
        fx.expected_controller_epoch=state_.controller_epoch;
        fx.cmd_reason="Friction identification post-alignment settled";
        phase_deadline_=now+kModeTimeout;
      } else if(state_.sysid_phase==S::SYSID_COMPLETING) {
        complete_sysid_unlocked();
        return fx;
      }
    } else if(state_.mode==S::MODE_RECOVERING) {
      if(state_.recovery_phase==S::RECOVERY_WAIT_NORMAL_MODE) {
        fx.ctc_cmd=CTC::Request::CMD_CLEAR_ESTOP;
        fx.expected_controller_epoch=state_.controller_epoch;
        fx.cmd_reason="Arm settled; clearing controller E-STOP";
        phase_deadline_=now+kModeTimeout;
      } else if(state_.recovery_phase==S::RECOVERY_SETTLING) {
        state_.mode=S::MODE_NORMAL;
        state_.recovery_phase=S::RECOVERY_NONE;
        state_.estop_latched=false;
        state_.estop_reason=S::ESTOP_NONE;
        state_.session_id=0;
        phase_deadline_=Clock::time_point{};
      }
    }
    changed_unlocked("Physical settling confirmed; sequence advanced");
  }
  return fx;
}
Supervisor::State Supervisor::snapshot() const {
  std::lock_guard<std::mutex> guard(mutex_);
  State state=state_;
  state.controller_fresh=fresh_controller_unlocked();
  state.joint_states_fresh=fresh_joint_unlocked();
  if(!state.controller_fresh) state.controller_mode=CS::MODE_UNKNOWN;
  state.motion_allowed=state.mode==S::MODE_NORMAL&&state.motion_phase==S::MOTION_IDLE&&
      state.controller_mode==CS::MODE_NORMAL&&state.controller_fresh&&
      state.joint_states_fresh&&!state.estop_latched;
  return state;
}
bool Supervisor::can_start_motion_unlocked(uint8_t context,uint64_t session,std::string& why) const {
  if(!active_key_.empty()||state_.motion_phase!=S::MOTION_IDLE) {why="Motion slot busy";return false;}
  if(!fresh_controller_unlocked()||!fresh_joint_unlocked()) {why="Missing fresh hardware observation";return false;}
  if(state_.waiting_for_settle) {why="Waiting for physical settling";return false;}
  if(state_.estop_latched) {why="ESTOP is latched";return false;}
  if(context==0) {
    if(state_.mode!=S::MODE_NORMAL||state_.controller_mode!=CS::MODE_NORMAL||session!=0) {
      why="Normal motion requires NORMAL mode and zero session";return false;
    }
  } else if(context==1) {
    if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
       !(state_.sysid_phase==S::SYSID_PRE_ALIGN||
         state_.sysid_phase==S::SYSID_POST_ALIGN||
         state_.sysid_phase==S::SYSID_RETURN_ALIGN)) {
      why="Friction identification phase/session does not permit alignment";return false;
    }
    if(state_.sysid_phase==S::SYSID_PRE_ALIGN&&state_.controller_mode!=CS::MODE_NORMAL) {
      why="Friction identification pre-alignment requires normal CTC mode";return false;
    }
    if(state_.sysid_phase==S::SYSID_POST_ALIGN&&state_.controller_mode!=CS::MODE_SYSID) {
      why="Friction identification post-alignment requires friction ID CTC mode";return false;
    }
    if(state_.sysid_phase==S::SYSID_RETURN_ALIGN&&state_.controller_mode!=CS::MODE_NORMAL) {
      why="Friction identification return alignment requires normal CTC mode";return false;
    }
  } else {why="Unknown motion context";return false;}
  return true;
}
bool Supervisor::reserve_motion(const std::string& key,uint8_t context,uint64_t session,
    std::string& why) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(!can_start_motion_unlocked(context,session,why)) return false;
  active_key_=key;
  phase_deadline_=Clock::time_point{};
  cancel_requested_=false;
  state_.motion_phase=S::MOTION_PLANNING;
  changed_unlocked("Motion admitted for planning");
  return true;
}
bool Supervisor::begin_excitation_validation(uint64_t session,std::string& why) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
      state_.sysid_phase!=S::SYSID_READY_FOR_EXCITATION||state_.waiting_for_settle||
      !active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
      state_.controller_mode!=CS::MODE_SYSID||Clock::now()>phase_deadline_) {
    why="Supervisor does not permit friction identification validation";return false;
  }
  state_.sysid_phase=S::SYSID_VALIDATING;
  phase_deadline_=Clock::now()+kValidationTimeout;
  changed_unlocked("Validating friction identification trajectory; arm remains at alignment pose");
  return true;
}
void Supervisor::end_excitation_validation(uint64_t session) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(state_.mode!=S::MODE_SYSID||state_.session_id!=session||
      state_.sysid_phase!=S::SYSID_VALIDATING) return;
  state_.sysid_phase=S::SYSID_READY_FOR_EXCITATION;
  phase_deadline_=Clock::now()+kDispatchTimeout;
  changed_unlocked("Friction identification validation rejected; waiting for client abort or retry");
}
bool Supervisor::reserve_excitation(uint64_t session,std::string& key,std::string& why) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
      state_.sysid_phase!=S::SYSID_VALIDATING||Clock::now()>phase_deadline_||
      state_.waiting_for_settle||!active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
      state_.controller_mode!=CS::MODE_SYSID) {
    why="Supervisor does not permit friction identification excitation";return false;
  }
  key="sysid-excitation:"+std::to_string(session);
  active_key_=key;cancel_requested_=false;
  phase_deadline_=Clock::time_point{};
  state_.excitation_start=builtin_interfaces::msg::Time{};
  state_.sysid_phase=S::SYSID_EXCITING;
  state_.motion_phase=S::MOTION_WAITING_CTC;
  changed_unlocked("Friction identification excitation dispatched");
  return true;
}
bool Supervisor::permitted(const std::string& key) const {
  std::lock_guard<std::mutex> guard(mutex_);
  if(cancel_requested_||active_key_!=key||!fresh_controller_unlocked()||!fresh_joint_unlocked())
    return false;
  if(state_.mode==S::MODE_NORMAL) return state_.controller_mode==CS::MODE_NORMAL;
  if(state_.mode==S::MODE_SYSID) {
    const auto p=state_.sysid_phase;
    if(p==S::SYSID_PRE_ALIGN||p==S::SYSID_RETURN_ALIGN)
      return state_.controller_mode==CS::MODE_NORMAL;
    return state_.controller_mode==CS::MODE_SYSID;
  }
  return false;
}
bool Supervisor::change_motion_phase(const std::string& key,uint8_t phase) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(active_key_!=key||cancel_requested_) return false;
  if(state_.motion_phase==phase) return true;
  if(!((state_.motion_phase==S::MOTION_PLANNING&&phase==S::MOTION_WAITING_CTC)||
       (state_.motion_phase==S::MOTION_WAITING_CTC&&phase==S::MOTION_EXECUTING)))
    return false;
  state_.motion_phase=phase;
  changed_unlocked("Motion phase advanced");
  return true;
}
bool Supervisor::request_cancel(const std::string& key) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(active_key_!=key||key.empty()) return false;
  cancel_requested_=true;
  state_.motion_phase=S::MOTION_CANCELING;
  changed_unlocked("Motion cancellation requested");
  return true;
}
Supervisor::Effects Supervisor::finish_motion(const std::string& key,bool success,
    int32_t code,const std::string& message) {
  std::lock_guard<std::mutex> guard(mutex_);
  Effects fx;
  if(active_key_!=key||key.empty()) return fx; // stale async result
  const bool was_cancel_requested=cancel_requested_;
  const bool authorized=success&&!cancel_requested_&&
    (state_.mode==S::MODE_NORMAL||state_.mode==S::MODE_SYSID);
  const bool excitation=key.rfind("sysid-excitation:",0)==0;
  clear_motion_unlocked();
  fx.terminal_success=authorized;
  if(authorized) {
    if(state_.mode==S::MODE_SYSID) {
      if(excitation&&state_.sysid_phase==S::SYSID_EXCITING) {
        state_.sysid_phase=S::SYSID_CALCULATING;
        phase_deadline_=Clock::now()+kCalculationTimeout;
      } else if(state_.sysid_phase==S::SYSID_PRE_ALIGN) {
        state_.sysid_phase=S::SYSID_WAIT_CONTROLLER_MODE;
        settle_unlocked();
      } else if(state_.sysid_phase==S::SYSID_POST_ALIGN) {
        state_.sysid_phase=S::SYSID_WAIT_NORMAL_MODE;
        settle_unlocked();
      } else if(state_.sysid_phase==S::SYSID_RETURN_ALIGN) {
        state_.sysid_phase=S::SYSID_COMPLETING;
        settle_unlocked();
      }
    }
    state_.last_error_code=1;
    changed_unlocked("Motion completed: "+message);
  } else {
    if(state_.mode!=S::MODE_ESTOP) state_.last_error_code=code;
    if(state_.mode==S::MODE_ESTOP) {
      changed_unlocked(); // Cancellation acknowledgement must not hide the fault.
    } else if(state_.mode==S::MODE_SYSID||state_.mode==S::MODE_RECOVERING) {
      fault_unlocked("Sequence motion failed: "+message,true,fx,S::ESTOP_TRAJECTORY);
    } else if(state_.mode==S::MODE_NORMAL) {
      if(was_cancel_requested||code==-8) changed_unlocked("Normal motion cancelled");
      else {
        state_.mode=S::MODE_SOFT_FAULT;
        changed_unlocked("Motion failed: "+message);
      }
    } else changed_unlocked("Motion terminated");
  }
  return fx;
}
Supervisor::Reply Supervisor::command(const Command::Request& req) {
  std::lock_guard<std::mutex> guard(mutex_);
  Reply r;
  r.planner_epoch=planner_epoch_;
  auto reject=[&](const std::string& why){r.message=why;};
  if(req.command!=C::Request::CMD_ESTOP&&
     req.expected_planner_epoch>0&&req.expected_planner_epoch!=planner_epoch_) {
    reject("Planner epoch mismatch: expected "+std::to_string(req.expected_planner_epoch)+
      ", current "+std::to_string(planner_epoch_));
    r.session_id=state_.session_id;
    r.revision=state_.revision;
    r.mode=state_.mode;
    return r;
  }
  if(req.command!=C::Request::CMD_ESTOP&&
     req.expected_revision>0&&req.expected_revision!=state_.revision) {
    reject("Revision mismatch: expected "+std::to_string(req.expected_revision)+
      ", current "+std::to_string(state_.revision));
    r.session_id=state_.session_id;
    r.revision=state_.revision;
    r.mode=state_.mode;
    return r;
  }
  switch(req.command) {
  case C::Request::CMD_ESTOP: {
    r.accepted=true;
    const uint8_t req_reason=
      req.reason=="SOFTWARE E-STOP PRESSED"?S::ESTOP_MANUAL:S::ESTOP_SUPERVISOR;
    // Preserve an already-latched specific safety cause (e.g. compliance or trajectory
    // fault) if a redundant manual E-STOP arrives, while allowing an operator E-STOP
    // to refine a generic controller E-STOP observed from the parallel direct CTC latch.
    const bool preserve_root_cause=
      state_.mode==S::MODE_ESTOP&&
      state_.estop_reason!=S::ESTOP_NONE&&
      state_.estop_reason!=S::ESTOP_MANUAL&&
      state_.estop_reason!=S::ESTOP_SUPERVISOR&&
      state_.estop_reason!=S::ESTOP_CONTROLLER_FAULT;
    if(preserve_root_cause) {
      r.effects.ctc_cmd=CTC::Request::CMD_ESTOP;
      r.effects.expected_controller_epoch=0;
      r.effects.cmd_reason=req.reason;
      changed_unlocked();
    } else {
      fault_unlocked("ESTOP: "+req.reason,true,r.effects,req_reason);
    }
    break;
  }
  case C::Request::CMD_JOINT_IMPEDANCE:
  case C::Request::CMD_CARTESIAN_IMPEDANCE:
  case C::Request::CMD_TEACH:
  case C::Request::CMD_EXIT_COMPLIANCE: {
    const bool compliant=state_.mode>=S::MODE_JOINT_IMPEDANCE&&state_.mode<=S::MODE_TEACH;
    const bool exiting=req.command==C::Request::CMD_EXIT_COMPLIANCE;
    const uint8_t target=exiting?CS::MODE_NORMAL:
      (req.command==C::Request::CMD_JOINT_IMPEDANCE?CS::MODE_JOINT_IMPEDANCE:
       (req.command==C::Request::CMD_CARTESIAN_IMPEDANCE?CS::MODE_CARTESIAN_IMPEDANCE:CS::MODE_TEACH));
    const bool same=compliant&&target==state_.controller_mode;
    if(req.session_id!=0||state_.estop_latched||!active_key_.empty()||
       !fresh_joint_unlocked()||!fresh_controller_unlocked()||
       !(compliant||(state_.mode==S::MODE_NORMAL&&state_.controller_mode==CS::MODE_NORMAL))||
       (exiting&&!compliant)) {
      reject("Compliance requires idle NORMAL/compliance mode and fresh feedback");break;
    }
    const int64_t now_ns=lite6_interfaces::shm::steady_now_ns();
    if(!exiting&&!same&&
       (stable_since_ns_<=0||
        !lite6_interfaces::shm::is_stamp_fresh(joint_sample_ns_,now_ns,kSettleFeedbackGapNs)||
        (joint_sample_ns_-stable_since_ns_)<kSettleDurationNs)) {
      reject("Release the arm and wait until stationary before changing compliance mode");break;
    }
    if(!exiting&&!lite6_interfaces::valid_compliance_settings(req.compliance)) {
      reject("Invalid compliance gains / sensitivity");break;
    }
    r.effects.ctc_cmd=exiting?CTC::Request::CMD_ENTER_NORMAL:
      (req.command==C::Request::CMD_JOINT_IMPEDANCE?CTC::Request::CMD_ENTER_JOINT_IMPEDANCE:
       (req.command==C::Request::CMD_CARTESIAN_IMPEDANCE?CTC::Request::CMD_ENTER_CARTESIAN_IMPEDANCE:
        CTC::Request::CMD_ENTER_TEACH));
    r.effects.has_compliance=!exiting;
    if(!exiting) r.effects.compliance=req.compliance;
    r.effects.expected_controller_epoch=state_.controller_epoch;
    r.effects.cmd_reason=req.reason;
    if(!same) {
      switch_unknown_deadline_=Clock::time_point{};
      pre_switch_mode_=state_.mode;
      compliance_target_=target;
      state_.mode=S::MODE_COMPLIANCE_SWITCHING;
      phase_deadline_=Clock::now()+kModeTimeout;
    }
    changed_unlocked(same?"Compliance settings accepted":"Waiting for compliance mode acknowledgement");
    r.accepted=true;
    break;
  }
  case C::Request::CMD_UPDATE_COMPLIANCE: {
    const bool compliant=state_.mode>=S::MODE_JOINT_IMPEDANCE&&state_.mode<=S::MODE_TEACH;
    if(!compliant||req.session_id!=0||state_.estop_latched||!active_key_.empty()||
       !fresh_joint_unlocked()||!fresh_controller_unlocked()) {
      reject("Compliance update requires active idle compliance mode and fresh feedback");break;
    }
    if(!lite6_interfaces::valid_compliance_settings(req.compliance)) {
      reject("Invalid compliance gains / sensitivity");break;
    }
    r.effects.ctc_cmd=CTC::Request::CMD_UPDATE_COMPLIANCE_SETTINGS;
    r.effects.has_compliance=true;
    r.effects.compliance=req.compliance;
    r.effects.expected_controller_epoch=state_.controller_epoch;
    r.effects.cmd_reason=req.reason;
    changed_unlocked("Updating live compliance settings");
    r.accepted=true;
    break;
  }
  case C::Request::CMD_SET_PAYLOAD:
    if(req.session_id!=0||!active_key_.empty()||!fresh_controller_unlocked()||
       state_.mode==S::MODE_INITIALIZING||state_.mode==S::MODE_COMPLIANCE_SWITCHING||
       state_.mode==S::MODE_SYSID||state_.controller_mode==CS::MODE_SYSID) {
      reject("Payload update requires idle non-identification state and fresh controller observation");break;
    }
    if(!lite6_interfaces::valid_payload_params(req.payload_mass,req.payload_com_tcp)) {
      reject("Payload parameters out of bounds");break;
    }
    r.effects.ctc_cmd=CTC::Request::CMD_SET_PAYLOAD;
    r.effects.has_payload=true;
    r.effects.payload_mass=req.payload_mass;
    r.effects.payload_com_tcp=req.payload_com_tcp;
    r.effects.expected_controller_epoch=state_.controller_epoch;
    r.effects.cmd_reason=req.reason;
    changed_unlocked("Updating payload model ("+format_fixed(req.payload_mass,3)+" kg)");
    r.accepted=true;
    break;
  case C::Request::CMD_BEGIN_RECOVERY:
    if(state_.mode!=S::MODE_ESTOP||!state_.estop_latched||req.session_id!=0||
       !active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
       state_.controller_mode!=CS::MODE_ESTOP) {
      reject(seen_controller_&&!fresh_controller_unlocked()?
        std::string("Recovery requires fresh controller feedback.\n")+kControllerLostHint:
        std::string("Recovery requires observed ESTOP, idle motion slot and fresh feedback"));
      break;
    }
    state_.mode=S::MODE_RECOVERING;
    state_.recovery_phase=S::RECOVERY_WAIT_NORMAL_MODE;
    state_.session_id=next_session_++;
    settle_unlocked();
    changed_unlocked("Waiting for stationary settle before clearing controller ESTOP");
    r.accepted=true;
    break;
  case C::Request::CMD_ACK_SOFT_FAULT:
    if(state_.mode!=S::MODE_SOFT_FAULT||
       state_.estop_latched||req.session_id!=0||!active_key_.empty()||
       !fresh_joint_unlocked()||!fresh_controller_unlocked()||
       state_.controller_mode!=CS::MODE_NORMAL) {
      reject("Cannot acknowledge while controller unhealthy or task running");break;
    }
    state_.mode=S::MODE_NORMAL;
    state_.last_error_code=0;
    changed_unlocked("Soft fault acknowledged");
    r.accepted=true;
    break;
  case C::Request::CMD_BEGIN_SYSID:
    if(state_.mode!=S::MODE_NORMAL||state_.estop_latched||req.session_id!=0||
       !active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
       state_.controller_mode!=CS::MODE_NORMAL) {
      reject("Friction identification requires idle NORMAL mode and fresh observations");break;
    }
    if(state_.payload_mass>1e-6) {
      reject("Friction identification is prohibited when payload is attached ("+
        format_fixed(state_.payload_mass,3)+" kg > 0.0 kg); reset payload to 0 kg first");
      break;
    }
    state_.mode=S::MODE_SYSID;
    state_.sysid_phase=S::SYSID_PRE_ALIGN;
    state_.session_id=next_session_++;
    state_.excitation_start=builtin_interfaces::msg::Time{};
    sysid_failure_.clear();
    settle_unlocked();
    changed_unlocked("Waiting for settling before friction identification pre-alignment");
    r.accepted=true;
    break;
  case C::Request::CMD_SYSID_CALCULATION_DONE:
  case C::Request::CMD_SYSID_CALCULATION_FAILED:
    if(state_.mode!=S::MODE_SYSID||req.session_id==0||req.session_id!=state_.session_id||
       state_.sysid_phase!=S::SYSID_CALCULATING||!active_key_.empty()) {
      reject("Calculation-done event out of phase/session");break;
    }
    if(req.command==C::Request::CMD_SYSID_CALCULATION_FAILED)
      sysid_failure_=req.reason.empty()?"Friction estimation failed":req.reason;
    state_.sysid_phase=S::SYSID_POST_ALIGN;
    settle_unlocked();
    changed_unlocked(sysid_failure_.empty()?"Waiting for friction identification post-alignment settling":
        "Estimation failed; controlled return to NORMAL: "+sysid_failure_);
    r.accepted=true;
    break;
  case C::Request::CMD_ABORT_SYSID:
    if(state_.mode!=S::MODE_SYSID||req.session_id==0||req.session_id!=state_.session_id) {
      reject("Invalid friction identification abort session");break;
    }
    fault_unlocked("Friction identification aborted: "+req.reason,true,r.effects);
    r.accepted=true;
    break;
  default: reject("Unknown supervisor event");break;
  }
  r.session_id=state_.session_id;
  r.revision=state_.revision;
  r.mode=state_.mode;
  if(r.accepted) r.message=state_.detail;
  return r;
}
std::string Supervisor::active_motion_key() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return active_key_;
}
bool Supervisor::controller_fresh() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return fresh_controller_unlocked();
}
} // namespace lite6_planner
