#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <algorithm>
#include <cmath>

namespace lite6_controllers {
namespace {
double interpolate_teach_level(const std::array<double, 5>& levels, double sensitivity) {
  const double pos=std::clamp(sensitivity,1.0,5.0)-1.0;
  const size_t idx=std::min<size_t>(3,static_cast<size_t>(std::floor(pos)));
  const double frac=pos-static_cast<double>(idx);
  return levels[idx]+frac*(levels[idx+1]-levels[idx]);
}

bool load_teach_levels(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,double min_val,bool strict_positive,double max_val,
    std::array<double,5>& out) {
  const auto values=node->get_parameter(name).as_double_array();
  if(values.size()!=5) {
    RCLCPP_ERROR(node->get_logger(),"%s must contain exactly five values (levels 1..5)",name.c_str());
    return false;
  }
  for(size_t i=0;i<5;++i) {
    const bool ok=std::isfinite(values[i])&&
      (strict_positive?values[i]>min_val:values[i]>=min_val)&&values[i]<=max_val;
    if(!ok) {
      RCLCPP_ERROR(node->get_logger(),"%s[%zu] out of valid range",name.c_str(),i);
      return false;
    }
    out[i]=values[i];
  }
  return true;
}
}  // namespace

bool Lite6CTCController::compliance_active() const {
  return system_state_==SystemState::JOINT_IMPEDANCE||
    system_state_==SystemState::CARTESIAN_IMPEDANCE||system_state_==SystemState::TEACH;
}
bool Lite6CTCController::configure_compliance() {
  const auto node=get_node();
  compliance_max_joint_velocity_=node->get_parameter("compliance.max_joint_velocity").as_double();
  compliance_hard_limit_tolerance_=node->get_parameter("compliance.hard_limit_tolerance").as_double();
  compliance_saturation_timeout_=node->get_parameter("compliance.saturation_timeout").as_double();
  compliance_entry_max_speed_=node->get_parameter("compliance.entry_max_speed").as_double();
  compliance_filter_time_constant_=node->get_parameter("compliance.filter_time_constant").as_double();
  compliance_cartesian_null_damping_=node->get_parameter("compliance.cartesian_null_damping").as_double();
  compliance_soft_limit_margin_=node->get_parameter("compliance.soft_limit_margin").as_double();
  compliance_soft_limit_stiffness_=node->get_parameter("compliance.soft_limit_stiffness").as_double();
  compliance_soft_limit_damping_=node->get_parameter("compliance.soft_limit_damping").as_double();
  compliance_torque_slew_rate_=node->get_parameter("compliance.torque_slew_rate").as_double();
  compliance_fv_scale_=node->get_parameter("compliance.friction_viscous_scale").as_double();
  compliance_fc_scale_=node->get_parameter("compliance.friction_coulomb_scale").as_double();
  compliance_fric_trans_speed_=node->get_parameter("compliance.friction_transition_speed").as_double();
  compliance_fric_quiet_speed_=node->get_parameter("compliance.friction_quiet_speed").as_double();
  const auto ext_lim=node->get_parameter("compliance.max_external_joint_torque").as_double_array();
  compliance_external_torque_timeout_=node->get_parameter("compliance.external_torque_timeout").as_double();

  auto positive=[](double v){return std::isfinite(v)&&v>0.0;};
  auto nonneg=[](double v){return std::isfinite(v)&&v>=0.0;};
  auto unit=[](double v){return std::isfinite(v)&&v>=0.0&&v<=1.0;};
  if(!positive(compliance_max_joint_velocity_)||!nonneg(compliance_hard_limit_tolerance_)||
     !positive(compliance_saturation_timeout_)||!positive(compliance_entry_max_speed_)||
     !positive(compliance_filter_time_constant_)||!nonneg(compliance_cartesian_null_damping_)||
     !nonneg(compliance_soft_limit_margin_)||!nonneg(compliance_soft_limit_stiffness_)||
     !nonneg(compliance_soft_limit_damping_)||!positive(compliance_torque_slew_rate_)||
     !unit(compliance_fv_scale_)||!unit(compliance_fc_scale_)||
     !positive(compliance_fric_trans_speed_)||!nonneg(compliance_fric_quiet_speed_)||
     !positive(compliance_external_torque_timeout_)||ext_lim.size()!=6||
     !std::all_of(ext_lim.begin(),ext_lim.end(),positive)) {
    RCLCPP_ERROR(node->get_logger(),"Invalid compliance safety or control parameter");
    return false;
  }
  for(size_t i=0;i<6;++i) compliance_max_external_torque_[i]=ext_lim[i];
  if(!load_teach_levels(node,"compliance.teach.damping_levels",0.0,false,100.0,teach_damping_levels_)||
     !load_teach_levels(node,"compliance.teach.assistance_levels",0.0,false,1.0,teach_assistance_levels_)||
     !load_teach_levels(node,"compliance.teach.friction_transition_levels",0.0,true,10.0,teach_friction_transition_levels_)||
     !load_teach_levels(node,"compliance.teach.quiet_speed_levels",0.0,false,10.0,teach_quiet_speed_levels_)) {
    return false;
  }

  compliance_frame_=pin_model_.getFrameId("link_eef");
  if(compliance_frame_>=static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) throw std::runtime_error("Missing compliance TCP frame");
  compliance_jacobian_.setZero(6,pin_model_.nv);
  compliance_sub_=node->create_subscription<lite6_interfaces::msg::ComplianceCommand>(
    "~/compliance_cmd",rclcpp::QoS(10),
    [this](const lite6_interfaces::msg::ComplianceCommand::SharedPtr msg) {
      if(msg->command<3||msg->command>5||!lite6_interfaces::valid_compliance_settings(msg->settings)) return;
      ModeCommand command(msg->command,++command_sequence_);
      command.settings=msg->settings;
      cmd_buffer_.writeFromNonRT(command);
    });
  return true;
}
void Lite6CTCController::process_compliance_command(double dt, int32_t command,
    const lite6_interfaces::msg::ComplianceSettings& settings) {
  // Process every envelope at most once: a rejected command must never become
  // effective later after ESTOP recovery. The existing recovery remains intact.
  if(current_command_sequence_!=compliance_sequence_) {
    compliance_sequence_=current_command_sequence_;
    if(command>=3&&command<=5&&(system_state_==SystemState::NORMAL||compliance_active())&&
       !execution_stop_latched_&&(!current_execution_||
       current_execution_->result.load()!=TrajectoryResult::RUNNING)&&
       (!current_impedance_execution_||
       current_impedance_execution_->result.load()!=ImpedanceTrajectoryResult::RUNNING)) {
      const auto desired=command==3?SystemState::JOINT_IMPEDANCE:
        (command==4?SystemState::CARTESIAN_IMPEDANCE:SystemState::TEACH);
      const bool entering=desired!=system_state_;
      if(entering&&dq_.cwiseAbs().maxCoeff()>=compliance_entry_max_speed_) return;
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
        compliance_external_torque_time_=0.0;
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
      if(q_[i]<pin_model_.lowerPositionLimit[i]-compliance_hard_limit_tolerance_) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_POSITION_LIMIT,i,q_[i],pin_model_.lowerPositionLimit[i]);
        request_execution_stop();break;
      }
      if(q_[i]>pin_model_.upperPositionLimit[i]+compliance_hard_limit_tolerance_) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_POSITION_LIMIT,i,q_[i],pin_model_.upperPositionLimit[i]);
        request_execution_stop();break;
      }
      if(std::abs(dq_[i])>compliance_max_joint_velocity_) {
        latch_estop_reason(CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT,i,dq_[i],compliance_max_joint_velocity_);
        request_execution_stop();break;
      }
    }
    int ext_joint=-1; double ext_val=0.0; double ext_limit=0.0;
    if(check_external_torque_limit(dt,compliance_max_external_torque_,
        compliance_external_torque_timeout_,compliance_external_torque_time_,
        ext_joint,ext_val,ext_limit)) {
      latch_estop_reason(CS::ESTOP_COMPLIANCE_EXTERNAL_TORQUE,ext_joint,ext_val,ext_limit);
      request_execution_stop();
    }
  }
}
void Lite6CTCController::compute_compliance(double dt) {
  // Only encoder position/velocity and model dynamics/gravity are used. No effort
  // state, MuJoCo contact force, applied force, or hidden friction state enters here.
  const double alpha=1.0-std::exp(-dt/compliance_filter_time_constant_);
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
    add_passive_friction_ff(compliance_fv_scale_,compliance_fc_scale_,
      compliance_fric_trans_speed_,compliance_fric_quiet_speed_,dq_,tau_cmd_);
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
    tau_cmd_-=compliance_cartesian_null_damping_*dq_;
    add_passive_friction_ff(compliance_fv_scale_,compliance_fc_scale_,
      compliance_fric_trans_speed_,compliance_fric_quiet_speed_,dq_,tau_cmd_);
  } else {
    // Interpolate configured sensitivity 1..5 profiles and fade out Coulomb
    // assistance near rest so low-speed bristle dynamics stay quiet.
    const double damping=interpolate_teach_level(teach_damping_levels_,s.sensitivity);
    const double assistance=interpolate_teach_level(teach_assistance_levels_,s.sensitivity);
    const double friction_transition=interpolate_teach_level(teach_friction_transition_levels_,s.sensitivity);
    const double quiet_speed=interpolate_teach_level(teach_quiet_speed_levels_,s.sensitivity);
    tau_cmd_-=damping*dq_;
    add_passive_friction_ff(assistance,assistance,friction_transition,quiet_speed,dq_,tau_cmd_);
  }
  bool saturated=false;
  int saturated_joint=-1;
  for(int i=0;i<6;++i) {
    // Preserve an equilibrium captured inside the margin (home is on joint3's
    // lower boundary). Do not push the arm simply because a mode was entered.
    // Hard-limit checks tolerate the configured allowance for solver/encoder noise.
    const double lo=std::min(pin_model_.lowerPositionLimit[i]+compliance_soft_limit_margin_,compliance_q_[i]);
    const double hi=std::max(pin_model_.upperPositionLimit[i]-compliance_soft_limit_margin_,compliance_q_[i]);
    if(q_[i]<lo) tau_cmd_[i]+=compliance_soft_limit_stiffness_*(lo-q_[i])-compliance_soft_limit_damping_*std::min(0.0,dq_[i]);
    if(q_[i]>hi) tau_cmd_[i]-=compliance_soft_limit_stiffness_*(q_[i]-hi)+compliance_soft_limit_damping_*std::max(0.0,dq_[i]);
    if(std::abs(tau_cmd_[i])>max_effort_[i]) {
      saturated=true;
      if(saturated_joint<0) saturated_joint=i;
    }
    const double max_step=compliance_torque_slew_rate_*dt;
    tau_cmd_[i]=std::clamp(tau_cmd_[i],compliance_last_tau_[i]-max_step,
      compliance_last_tau_[i]+max_step);
    compliance_last_tau_[i]=std::clamp(tau_cmd_[i],-max_effort_[i],max_effort_[i]);
  }
  compliance_saturation_time_=saturated?compliance_saturation_time_+dt:0.0;
  // Apply braking on the next cycle, so this cycle retains force-mode semantics.
  if(compliance_saturation_time_>compliance_saturation_timeout_) {
    compliance_saturation_estop_pending_=true;
    pending_estop_joint_=static_cast<int8_t>(saturated_joint);
    pending_estop_value_=compliance_saturation_time_;
    pending_estop_limit_=compliance_saturation_timeout_;
    emergency_pending_.store(true);
  }
}
}
