#include "lite6_planner/supervisor.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace lite6_planner {
namespace {
using S=lite6_interfaces::msg::SupervisorState;
using C=lite6_interfaces::srv::SupervisorCommand;
using CS=lite6_interfaces::msg::ControllerStatus;
constexpr auto kObservationTimeout=std::chrono::milliseconds(800);
constexpr auto kModeTimeout=std::chrono::seconds(8);
constexpr auto kSettleDuration=std::chrono::milliseconds(300);
constexpr auto kSettleFeedbackGap=std::chrono::milliseconds(100);
constexpr auto kDispatchTimeout=std::chrono::seconds(15);
constexpr auto kCalculationTimeout=std::chrono::seconds(60);
constexpr double kSettleSpeed=0.01; // Stricter than the planner's 0.02 rad/s admission guard.
}

Supervisor::Supervisor() {
  // Session identifiers are correlation tokens, not authentication secrets.
  std::random_device rd;
  next_session_=(static_cast<uint64_t>(rd())<<32) | static_cast<uint64_t>(rd());
  if(next_session_==0) next_session_=1;
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
  return seen_controller_&&(Clock::now()-controller_time_<kObservationTimeout);
}
bool Supervisor::fresh_joint_unlocked() const {
  return seen_joint_&&(Clock::now()-joint_time_<kObservationTimeout);
}
void Supervisor::observe_joint_state(double max_speed) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto now=Clock::now();
  if(!seen_joint_||now-joint_time_>kSettleFeedbackGap||
      !std::isfinite(max_speed)||max_speed>kSettleSpeed) stable_since_=Clock::time_point{};
  if(std::isfinite(max_speed)&&max_speed<=kSettleSpeed&&stable_since_==Clock::time_point{})
    stable_since_=now;
  seen_joint_=true; joint_time_=now;
  if(state_.mode==S::MODE_INITIALIZING&&fresh_controller_unlocked()&&
     state_.controller_mode==CS::MODE_NORMAL) {
    state_.mode=S::MODE_NORMAL;
    changed_unlocked("Ready");
  }
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
  stable_since_=Clock::time_point{};
  phase_deadline_=Clock::now()+kModeTimeout;
}
void Supervisor::complete_sysid_unlocked() {
  state_.mode=S::MODE_NORMAL;
  state_.sysid_phase=S::SYSID_NONE;
  state_.session_id=0;
  state_.waiting_for_settle=false;
  phase_deadline_=Clock::time_point{};
  state_.last_error_code=sysid_failure_.empty()?1:S::ERROR_SYSID_ESTIMATION;
  changed_unlocked(sysid_failure_.empty()?"SysID completed; NORMAL and stationary":
      "SysID estimation failed; NORMAL restored: "+sysid_failure_);
}
void Supervisor::clear_motion_unlocked() {
  active_key_.clear();cancel_requested_=false;
  state_.motion_phase=S::MOTION_IDLE;
}
void Supervisor::fault_unlocked(const std::string& reason,bool estop,Effects& effect) {
  if(!active_key_.empty()) {
    effect.cancel_active=true;effect.cancel_key=active_key_;
    cancel_requested_=true;
    state_.motion_phase=S::MOTION_CANCELING;
  }
  if(estop) {
    state_.mode=S::MODE_ESTOP;
    state_.estop_latched=true;
    effect.ctc_command=1;
  } else state_.mode=S::MODE_SOFT_FAULT;
  state_.sysid_phase=S::SYSID_NONE;
  state_.recovery_phase=S::RECOVERY_NONE;
  state_.session_id=0;
  state_.waiting_for_settle=false;
  phase_deadline_=Clock::time_point{};
  state_.last_error_code=-7;
  changed_unlocked(reason);
}
Supervisor::Effects Supervisor::observe_controller(uint8_t mode) {
  std::lock_guard<std::mutex> guard(mutex_);
  Effects fx;
  if(mode!=CS::MODE_NORMAL&&mode!=CS::MODE_ESTOP&&mode!=CS::MODE_SYSID) return fx;
  const auto previous=state_.controller_mode;
  seen_controller_=true;controller_time_=Clock::now();
  state_.controller_mode=mode;
  if(mode==CS::MODE_ESTOP&&state_.mode!=S::MODE_ESTOP&&state_.mode!=S::MODE_RECOVERING) {
    fault_unlocked("CTC reports ESTOP",true,fx);
    fx.ctc_command=-1; // already in ESTOP; do not echo a redundant command
  } else if(mode==CS::MODE_SYSID&&state_.mode!=S::MODE_SYSID) {
    fault_unlocked("CTC entered SYS_ID without supervisor authorization",true,fx);
  } else if(state_.mode==S::MODE_INITIALIZING&&mode==CS::MODE_NORMAL&&fresh_joint_unlocked()) {
    state_.mode=S::MODE_NORMAL;changed_unlocked("Ready");
  } else if(state_.mode==S::MODE_SYSID&&state_.sysid_phase==S::SYSID_WAIT_CONTROLLER_MODE&&
      !state_.waiting_for_settle&&mode==CS::MODE_SYSID) {
    state_.sysid_phase=S::SYSID_READY_FOR_EXCITATION;
    settle_unlocked();
    changed_unlocked("CTC confirmed SYS_ID; waiting for physical settling");
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
Supervisor::Effects Supervisor::watchdog() {
  std::lock_guard<std::mutex> guard(mutex_);
  Effects fx;
  if(state_.mode==S::MODE_INITIALIZING) return fx;
  if(state_.mode==S::MODE_ESTOP||state_.mode==S::MODE_FAULT) return fx;
  if(!fresh_controller_unlocked()||!fresh_joint_unlocked()) {
    fault_unlocked("CTC / joint_states heartbeat expired",true,fx);
    return fx;
  }
  if(state_.mode==S::MODE_NORMAL&&state_.controller_mode!=CS::MODE_NORMAL) {
    fault_unlocked("Unexpected controller mode in NORMAL operation",true,fx);
    return fx;
  }
  if(state_.mode==S::MODE_SYSID) {
    const auto phase=state_.sysid_phase;
    const bool requires_normal=(phase==S::SYSID_PRE_ALIGN||phase==S::SYSID_RETURN_ALIGN||
       phase==S::SYSID_COMPLETING||
       (phase==S::SYSID_WAIT_CONTROLLER_MODE&&state_.waiting_for_settle));
    const bool requires_sysid=(phase==S::SYSID_READY_FOR_EXCITATION||
       phase==S::SYSID_EXCITING||phase==S::SYSID_CALCULATING||phase==S::SYSID_POST_ALIGN||
       (phase==S::SYSID_WAIT_NORMAL_MODE&&state_.waiting_for_settle));
    if((requires_normal&&state_.controller_mode!=CS::MODE_NORMAL)||
       (requires_sysid&&state_.controller_mode!=CS::MODE_SYSID)) {
      fault_unlocked("Unexpected CTC mode in SysID phase",true,fx);
      return fx;
    }
  }
  if(state_.mode==S::MODE_RECOVERING&&
     state_.recovery_phase==S::RECOVERY_SYNC_REFERENCE&&
     state_.controller_mode!=CS::MODE_ESTOP) {
    fault_unlocked("CTC mode mismatch during recovery sync",true,fx);
    return fx;
  }
  if(state_.mode==S::MODE_RECOVERING&&state_.recovery_phase==S::RECOVERY_SETTLING&&
      state_.controller_mode!=CS::MODE_NORMAL) {
    fault_unlocked("CTC mode mismatch during recovery settling",true,fx);
    return fx;
  }
  const auto now=Clock::now();
  if(phase_deadline_!=Clock::time_point{}&&now>phase_deadline_) {
    fault_unlocked(state_.waiting_for_settle?"Physical settling timed out":
        "Sequence phase timed out (mode acknowledgement / client response)",true,fx);
    return fx;
  }
  if(state_.waiting_for_settle&&stable_since_!=Clock::time_point{}&&
      now-stable_since_>=kSettleDuration&&now-joint_time_<kSettleFeedbackGap) {
    state_.waiting_for_settle=false;
    phase_deadline_=now+kDispatchTimeout;
    if(state_.mode==S::MODE_SYSID) {
      if(state_.sysid_phase==S::SYSID_WAIT_CONTROLLER_MODE) {
        fx.ctc_command=2;
        phase_deadline_=now+kModeTimeout;
      } else if(state_.sysid_phase==S::SYSID_WAIT_NORMAL_MODE) {
        fx.ctc_command=0;
        phase_deadline_=now+kModeTimeout;
      } else if(state_.sysid_phase==S::SYSID_COMPLETING) {
        complete_sysid_unlocked();
        return fx;
      }
    } else if(state_.mode==S::MODE_RECOVERING) {
      if(state_.recovery_phase==S::RECOVERY_WAIT_NORMAL_MODE) {
        fx.ctc_command=0;
        phase_deadline_=now+kModeTimeout;
      } else if(state_.recovery_phase==S::RECOVERY_SETTLING) {
        state_.mode=S::MODE_NORMAL;
        state_.recovery_phase=S::RECOVERY_NONE;
        state_.estop_latched=false;
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
  if(state_.estop_latched && context!=2) {why="ESTOP is latched";return false;}
  if(context==0) {
    if(state_.mode!=S::MODE_NORMAL||state_.controller_mode!=CS::MODE_NORMAL||session!=0) {
      why="Normal motion requires NORMAL mode and zero session";return false;
    }
  } else if(context==1) {
    if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
       !(state_.sysid_phase==S::SYSID_PRE_ALIGN||
         state_.sysid_phase==S::SYSID_POST_ALIGN||
         state_.sysid_phase==S::SYSID_RETURN_ALIGN)) {
      why="SysID phase/session does not permit alignment";return false;
    }
    if(state_.sysid_phase==S::SYSID_PRE_ALIGN&&state_.controller_mode!=CS::MODE_NORMAL) {
      why="SysID pre-alignment requires normal CTC mode";return false;
    }
    if(state_.sysid_phase==S::SYSID_POST_ALIGN&&state_.controller_mode!=CS::MODE_SYSID) {
      why="SysID post-alignment requires SYS_ID CTC mode";return false;
    }
    if(state_.sysid_phase==S::SYSID_RETURN_ALIGN&&state_.controller_mode!=CS::MODE_NORMAL) {
      why="SysID return alignment requires normal CTC mode";return false;
    }
  } else if(context==2) {
    if(state_.mode!=S::MODE_RECOVERING||session==0||session!=state_.session_id||
       state_.recovery_phase!=S::RECOVERY_SYNC_REFERENCE||
       state_.controller_mode!=CS::MODE_ESTOP) {
      why="Recovery phase/session does not permit reference sync";return false;
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
bool Supervisor::reserve_excitation(uint64_t session,std::string& key,std::string& why) {
  std::lock_guard<std::mutex> guard(mutex_);
  if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
      state_.sysid_phase!=S::SYSID_READY_FOR_EXCITATION||
      state_.waiting_for_settle||!active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
      state_.controller_mode!=CS::MODE_SYSID) {
    why="Supervisor does not permit SysID excitation";return false;
  }
  key="sysid-excitation:"+std::to_string(session);
  active_key_=key;cancel_requested_=false;
  phase_deadline_=Clock::time_point{};
  state_.excitation_start=builtin_interfaces::msg::Time{};
  state_.sysid_phase=S::SYSID_EXCITING;
  state_.motion_phase=S::MOTION_WAITING_CTC;
  changed_unlocked("SysID excitation dispatched");
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
  if(state_.mode==S::MODE_RECOVERING)
    return state_.recovery_phase==S::RECOVERY_SYNC_REFERENCE&&
           state_.controller_mode==CS::MODE_ESTOP;
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
    (state_.mode==S::MODE_NORMAL||state_.mode==S::MODE_SYSID||
     state_.mode==S::MODE_RECOVERING);
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
    } else if(state_.mode==S::MODE_RECOVERING&&
              state_.recovery_phase==S::RECOVERY_SYNC_REFERENCE) {
      state_.recovery_phase=S::RECOVERY_WAIT_NORMAL_MODE;
      settle_unlocked();
    }
    state_.last_error_code=1;
    changed_unlocked("Motion completed: "+message);
  } else {
    state_.last_error_code=code;
    if(state_.mode==S::MODE_ESTOP) {
      changed_unlocked("Motion stopped under ESTOP");
    } else if(state_.mode==S::MODE_SYSID||state_.mode==S::MODE_RECOVERING) {
      fault_unlocked("Sequence motion failed: "+message,true,fx);
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
Supervisor::Reply Supervisor::command(uint8_t event,uint64_t session,const std::string& reason) {
  std::lock_guard<std::mutex> guard(mutex_);
  Reply r;
  auto reject=[&](const std::string& why){r.message=why;};
  switch(event) {
  case C::Request::CMD_ESTOP:
    r.accepted=true;
    fault_unlocked("ESTOP: "+reason,true,r.effects);
    break;
  case C::Request::CMD_BEGIN_RECOVERY:
    if(state_.mode!=S::MODE_ESTOP||!state_.estop_latched||session!=0||
       !active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
       state_.controller_mode!=CS::MODE_ESTOP) {
      reject("Recovery requires observed ESTOP, idle motion slot and fresh feedback");break;
    }
    state_.mode=S::MODE_RECOVERING;
    state_.recovery_phase=S::RECOVERY_SYNC_REFERENCE;
    state_.session_id=next_session_++;
    settle_unlocked();
    changed_unlocked("Waiting for recovery settling before reference synchronization");
    r.accepted=true;
    break;
  case C::Request::CMD_ACK_SOFT_FAULT:
    if(state_.mode!=S::MODE_SOFT_FAULT||session!=0||!active_key_.empty()||
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
    if(state_.mode!=S::MODE_NORMAL||state_.estop_latched||session!=0||
       !active_key_.empty()||!fresh_joint_unlocked()||!fresh_controller_unlocked()||
       state_.controller_mode!=CS::MODE_NORMAL) {
      reject("SysID requires idle NORMAL mode and fresh observations");break;
    }
    state_.mode=S::MODE_SYSID;
    state_.sysid_phase=S::SYSID_PRE_ALIGN;
    state_.session_id=next_session_++;
    state_.excitation_start=builtin_interfaces::msg::Time{};
    sysid_failure_.clear();
    settle_unlocked();
    changed_unlocked("Waiting for settling before SysID pre-alignment");
    r.accepted=true;
    break;
  case C::Request::CMD_SYSID_CALCULATION_DONE:
  case C::Request::CMD_SYSID_CALCULATION_FAILED:
    if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id||
       state_.sysid_phase!=S::SYSID_CALCULATING||!active_key_.empty()) {
      reject("Calculation-done event out of phase/session");break;
    }
    if(event==C::Request::CMD_SYSID_CALCULATION_FAILED)
      sysid_failure_=reason.empty()?"Parameter estimation failed":reason;
    state_.sysid_phase=S::SYSID_POST_ALIGN;
    settle_unlocked();
    changed_unlocked(sysid_failure_.empty()?"Waiting for SysID post-alignment settling":
        "Estimation failed; controlled return to NORMAL: "+sysid_failure_);
    r.accepted=true;
    break;
  case C::Request::CMD_ABORT_SYSID:
    if(state_.mode!=S::MODE_SYSID||session==0||session!=state_.session_id) {
      reject("Invalid SysID abort session");break;
    }
    fault_unlocked("SysID aborted: "+reason,true,r.effects);
    r.accepted=true;
    break;
  default: reject("Unknown supervisor event");break;
  }
  r.session_id=state_.session_id;
  r.revision=state_.revision;
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
