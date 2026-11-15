#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <algorithm>
#include <cmath>

namespace lite6_controllers {
bool Lite6CTCController::compliance_active() const {
  return system_state_==SystemState::JOINT_IMPEDANCE||
    system_state_==SystemState::CARTESIAN_IMPEDANCE||system_state_==SystemState::TEACH;
}
void Lite6CTCController::configure_compliance() {
  compliance_frame_=pin_model_.getFrameId("link_eef");
  if(compliance_frame_>=static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) throw std::runtime_error("Missing compliance TCP frame");
  compliance_jacobian_.setZero(6,pin_model_.nv);
  compliance_sub_=get_node()->create_subscription<lite6_interfaces::msg::ComplianceCommand>(
    "~/compliance_cmd",rclcpp::QoS(10),
    [this](const lite6_interfaces::msg::ComplianceCommand::SharedPtr msg) {
      if(msg->command<3||msg->command>5||!lite6_interfaces::valid_compliance_settings(msg->settings)) return;
      ModeCommand command(msg->command,++command_sequence_);
      command.settings=msg->settings;
      cmd_buffer_.writeFromNonRT(command);
    });
}
void Lite6CTCController::process_compliance_command(int32_t command,
    const lite6_interfaces::msg::ComplianceSettings& settings) {
  // Process every envelope at most once: a rejected command must never become
  // effective later after ESTOP recovery. The existing recovery remains intact.
  if(current_command_sequence_!=compliance_sequence_) {
    compliance_sequence_=current_command_sequence_;
    if(command>=3&&command<=5&&(system_state_==SystemState::NORMAL||compliance_active())&&
       !execution_stop_latched_&&(!current_execution_||
       current_execution_->result.load()!=TrajectoryResult::RUNNING)) {
      const auto desired=command==3?SystemState::JOINT_IMPEDANCE:
        (command==4?SystemState::CARTESIAN_IMPEDANCE:SystemState::TEACH);
      const bool entering=desired!=system_state_;
      if(entering&&dq_.cwiseAbs().maxCoeff()>=0.01) return;
      compliance_settings_=settings;
      if(entering||settings.recapture) {
        compliance_q_=q_;
        pinocchio::forwardKinematics(pin_model_,pin_data_,dynamics_q_);
        pinocchio::updateFramePlacements(pin_model_,pin_data_);
        compliance_pose_=pin_data_.oMf[compliance_frame_];
      }
      if(entering) {
        compliance_filtered_=settings;
        compliance_last_tau_=tau_cmd_;
        compliance_saturation_time_=0.0;
        compliance_saturation_estop_pending_=false;
        integral_effort_.setZero();
      }
      system_state_=desired;
    } else if(command==6&&compliance_active()) {
      system_state_=SystemState::COMPLIANCE_STOPPING;
      estop_q_=q_;estop_dq_=dq_;
      integral_effort_.setZero();
    }
  }
  if(system_state_==SystemState::COMPLIANCE_STOPPING&&
     estop_dq_.cwiseAbs().maxCoeff()<1e-4&&dq_.cwiseAbs().maxCoeff()<0.01) {
    // Resume NORMAL at the measured resting pose, never at the old equilibrium.
    for(size_t i=0;i<6;++i) {
      reference_interfaces_[i]=q_[i];
      reference_interfaces_[6+i]=0.0;reference_interfaces_[12+i]=0.0;
    }
    clear_estop_reason();
    system_state_=SystemState::NORMAL;
  }
  if(compliance_active()) {
    using CS=lite6_interfaces::msg::ControllerStatus;
    for(int i=0;i<6;++i) {
      if(q_[i]<pin_model_.lowerPositionLimit[i]-0.01) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_POSITION_LIMIT,i,q_[i],pin_model_.lowerPositionLimit[i]);
        request_execution_stop();break;
      }
      if(q_[i]>pin_model_.upperPositionLimit[i]+0.01) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_POSITION_LIMIT,i,q_[i],pin_model_.upperPositionLimit[i]);
        request_execution_stop();break;
      }
      if(std::abs(dq_[i])>1.5) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT,i,dq_[i],1.5);
        request_execution_stop();break;
      }
    }
  }
}
void Lite6CTCController::compute_compliance(double dt) {
  // Only encoder position/velocity and model gravity are used. No effort state,
  // MuJoCo contact force, applied force, or hidden friction state enters here.
  const double alpha=1.0-std::exp(-dt/0.15);
  auto smooth=[alpha](double& value,double target){value+=alpha*(target-value);};
  auto& s=compliance_filtered_;
  smooth(s.joint_stiffness,compliance_settings_.joint_stiffness);
  smooth(s.joint_damping,compliance_settings_.joint_damping);
  smooth(s.translation_stiffness,compliance_settings_.translation_stiffness);
  smooth(s.translation_damping,compliance_settings_.translation_damping);
  smooth(s.rotation_stiffness,compliance_settings_.rotation_stiffness);
  smooth(s.rotation_damping,compliance_settings_.rotation_damping);
  smooth(s.sensitivity,compliance_settings_.sensitivity);
  tau_cmd_=pinocchio::computeGeneralizedGravity(pin_model_,pin_data_,dynamics_q_).head(6);
  a_d_.setZero();integral_effort_.setZero();
  if(system_state_==SystemState::JOINT_IMPEDANCE) {
    tau_cmd_+=s.joint_stiffness*(compliance_q_-q_)-s.joint_damping*dq_;
  } else if(system_state_==SystemState::CARTESIAN_IMPEDANCE) {
    pinocchio::computeJointJacobians(pin_model_,pin_data_,dynamics_q_);
    pinocchio::updateFramePlacements(pin_model_,pin_data_);
    compliance_jacobian_.setZero();
    pinocchio::getFrameJacobian(pin_model_,pin_data_,compliance_frame_,
      pinocchio::LOCAL_WORLD_ALIGNED,compliance_jacobian_);
    const auto& pose=pin_data_.oMf[compliance_frame_];
    Eigen::Matrix<double,6,1> error, wrench;
    error.head<3>()=compliance_pose_.translation()-pose.translation();
    error.tail<3>()=pinocchio::log3(compliance_pose_.rotation()*pose.rotation().transpose());
    const Eigen::Matrix<double,6,1> velocity=compliance_jacobian_*dynamics_dq_;
    wrench.head<3>()=s.translation_stiffness*error.head<3>()-s.translation_damping*velocity.head<3>();
    wrench.tail<3>()=s.rotation_stiffness*error.tail<3>()-s.rotation_damping*velocity.tail<3>();
    tau_cmd_.noalias()+=compliance_jacobian_.leftCols(6).transpose()*wrench;
    // Small joint damping also damps task-null directions near singularities.
    tau_cmd_-=0.15*dq_;
  } else {
    // Keep damping and friction assistance conservative across sensitivity
    // levels 1..5, and fade out Coulomb assistance near rest everywhere so
    // low-speed bristle dynamics cannot self-excite jitter at any level.
    const double light=(std::min(s.sensitivity,3.0)-1.0)/4.0;
    const double high=std::clamp((s.sensitivity-3.0)/2.0,0.0,1.0);
    const double damping=4.0-3.0*light-0.5*high;
    const double assistance=0.35+0.5*light+0.05*high;
    const double friction_transition=0.06;
    const double quiet_speed=0.06;
    for(int i=0;i<6;++i) {
      // Deliberately bounded below full compensation. At rest there is no
      // guessed force direction; the user still overcomes static friction.
      // Fade out Coulomb assistance near rest while preserving viscous
      // compensation and positive damping across all sensitivity levels.
      const double ramp=std::clamp((std::abs(dq_[i])-quiet_speed)/quiet_speed,0.0,1.0);
      const double coulomb_gate=ramp*ramp*(3.0-2.0*ramp);
      tau_cmd_[i]+=-damping*dq_[i]+assistance*(friction_v_[i]*dq_[i]+
        coulomb_gate*friction_c_[i]*std::tanh(dq_[i]/friction_transition));
    }
  }
  bool saturated=false;
  int saturated_joint=-1;
  for(int i=0;i<6;++i) {
    // Preserve an equilibrium captured inside the margin (home is on joint3's
    // lower boundary). Do not push the arm simply because a mode was entered.
    // Hard-limit checks tolerate 0.01 rad for solver compliance/encoder noise.
    const double lo=std::min(pin_model_.lowerPositionLimit[i]+0.15,compliance_q_[i]);
    const double hi=std::max(pin_model_.upperPositionLimit[i]-0.15,compliance_q_[i]);
    if(q_[i]<lo) tau_cmd_[i]+=80.0*(lo-q_[i])-5.0*std::min(0.0,dq_[i]);
    if(q_[i]>hi) tau_cmd_[i]-=80.0*(q_[i]-hi)+5.0*std::max(0.0,dq_[i]);
    if(std::abs(tau_cmd_[i])>max_effort_[i]) {
      saturated=true;
      if(saturated_joint<0) saturated_joint=i;
    }
    tau_cmd_[i]=std::clamp(tau_cmd_[i],compliance_last_tau_[i]-100.0*dt,
      compliance_last_tau_[i]+100.0*dt);
    compliance_last_tau_[i]=std::clamp(tau_cmd_[i],-max_effort_[i],max_effort_[i]);
  }
  compliance_saturation_time_=saturated?compliance_saturation_time_+dt:0.0;
  // Apply braking on the next cycle, so this cycle retains force-mode semantics.
  if(compliance_saturation_time_>saturation_timeout_) {
    compliance_saturation_estop_pending_=true;
    pending_estop_joint_=static_cast<int8_t>(saturated_joint);
    pending_estop_value_=compliance_saturation_time_;
    pending_estop_limit_=saturation_timeout_;
    emergency_pending_.store(true);
  }
}
}
