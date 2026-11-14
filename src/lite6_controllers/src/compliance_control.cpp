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
    system_state_=SystemState::NORMAL;
  }
  if(compliance_active()) {
    for(int i=0;i<6;++i) {
      if(q_[i]<pin_model_.lowerPositionLimit[i]-0.01||q_[i]>pin_model_.upperPositionLimit[i]+0.01||
         std::abs(dq_[i])>1.5) {
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
    const double light=(s.sensitivity-1.0)/4.0;
    const double damping=4.0-3.0*light;
    const double assistance=0.35+0.5*light;
    for(int i=0;i<6;++i) {
      // Deliberately bounded below full compensation. At rest there is no
      // guessed force direction; the user still overcomes static friction.
      tau_cmd_[i]+=-damping*dq_[i]+assistance*(friction_v_[i]*dq_[i]+
        friction_c_[i]*std::tanh(dq_[i]/0.03));
    }
  }
  bool saturated=false;
  for(int i=0;i<6;++i) {
    // Preserve an equilibrium captured inside the margin (home is on joint3's
    // lower boundary). Do not push the arm simply because a mode was entered.
    // Hard-limit checks tolerate 0.01 rad for solver compliance/encoder noise.
    const double lo=std::min(pin_model_.lowerPositionLimit[i]+0.15,compliance_q_[i]);
    const double hi=std::max(pin_model_.upperPositionLimit[i]-0.15,compliance_q_[i]);
    if(q_[i]<lo) tau_cmd_[i]+=80.0*(lo-q_[i])-5.0*std::min(0.0,dq_[i]);
    if(q_[i]>hi) tau_cmd_[i]-=80.0*(q_[i]-hi)+5.0*std::max(0.0,dq_[i]);
    saturated=saturated||std::abs(tau_cmd_[i])>max_effort_[i];
    tau_cmd_[i]=std::clamp(tau_cmd_[i],compliance_last_tau_[i]-100.0*dt,
      compliance_last_tau_[i]+100.0*dt);
    compliance_last_tau_[i]=std::clamp(tau_cmd_[i],-max_effort_[i],max_effort_[i]);
  }
  compliance_saturation_time_=saturated?compliance_saturation_time_+dt:0.0;
  // Apply braking on the next cycle, so this cycle retains force-mode semantics.
  if(compliance_saturation_time_>saturation_timeout_) emergency_pending_.store(true);
}
}
