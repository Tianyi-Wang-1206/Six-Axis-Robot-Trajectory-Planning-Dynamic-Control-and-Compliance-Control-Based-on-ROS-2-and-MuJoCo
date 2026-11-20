#pragma once
#include <array>
#include <cmath>
#include "lite6_interfaces/motion_contract.hpp"
#include "lite6_interfaces/msg/compliance_settings.hpp"
#include "lite6_interfaces/msg/impedance_trajectory_point.hpp"
#include "lite6_interfaces/srv/controller_command.hpp"

namespace lite6_interfaces {

inline bool bounded_finite(double v, double lo, double hi) {
  return std::isfinite(v) && v >= lo && v <= hi;
}

inline bool valid_compliance_settings(const msg::ComplianceSettings& s) {
  return bounded_finite(s.joint_stiffness, contract::kMinJointStiffness, contract::kMaxJointStiffness) &&
    bounded_finite(s.joint_damping, contract::kMinJointDamping, contract::kMaxJointDamping) &&
    bounded_finite(s.translation_stiffness, contract::kMinTranslationStiffness, contract::kMaxTranslationStiffness) &&
    bounded_finite(s.translation_damping, contract::kMinTranslationDamping, contract::kMaxTranslationDamping) &&
    bounded_finite(s.rotation_stiffness, contract::kMinRotationStiffness, contract::kMaxRotationStiffness) &&
    bounded_finite(s.rotation_damping, contract::kMinRotationDamping, contract::kMaxRotationDamping) &&
    bounded_finite(s.sensitivity, contract::kMinComplianceSensitivity, contract::kMaxComplianceSensitivity) &&
    bounded_finite(s.nullspace_stiffness, contract::kMinNullspaceStiffness, contract::kMaxNullspaceStiffness) &&
    bounded_finite(s.nullspace_damping, contract::kMinNullspaceDamping, contract::kMaxNullspaceDamping);
}

inline bool valid_impedance_trajectory_gains(const msg::ImpedanceTrajectoryPoint& p) {
  for (int i = 0; i < 3; ++i) {
    if (!bounded_finite(p.translation_stiffness[i], contract::kMinTranslationStiffness, contract::kMaxTranslationStiffness) ||
        !bounded_finite(p.translation_damping[i], contract::kMinTranslationDamping, contract::kMaxTranslationDamping) ||
        !bounded_finite(p.rotation_stiffness[i], contract::kMinRotationStiffness, contract::kMaxRotationStiffness) ||
        !bounded_finite(p.rotation_damping[i], contract::kMinRotationDamping, contract::kMaxRotationDamping) ||
        !bounded_finite(p.wrench_ff[i], -contract::kMaxFeedforwardForceN, contract::kMaxFeedforwardForceN) ||
        !bounded_finite(p.wrench_ff[i + 3], -contract::kMaxFeedforwardTorqueNm, contract::kMaxFeedforwardTorqueNm)) {
      return false;
    }
  }
  return bounded_finite(p.nullspace_damping, contract::kMinNullspaceDamping, contract::kMaxNullspaceDamping) &&
    bounded_finite(p.nullspace_stiffness, contract::kMinNullspaceStiffness, contract::kMaxNullspaceStiffness);
}

inline bool valid_transition_time(double transition_time_sec) {
  return bounded_finite(
    transition_time_sec,
    contract::kMinTransitionTimeSec,
    contract::kMaxTransitionTimeSec);
}

inline bool valid_payload_params(double mass_kg, const std::array<double, 3>& com_tcp) {
  if (!bounded_finite(mass_kg, contract::kMinPayloadMassKg, contract::kMaxPayloadMassKg)) {
    return false;
  }
  double sq_norm = 0.0;
  for (double c : com_tcp) {
    if (!std::isfinite(c)) {
      return false;
    }
    sq_norm += c * c;
  }
  return sq_norm <= (contract::kMaxPayloadComRadiusM * contract::kMaxPayloadComRadiusM);
}

inline bool valid_controller_command(const srv::ControllerCommand::Request& req) {
  using Cmd = srv::ControllerCommand::Request;
  switch (req.command) {
    case Cmd::CMD_ENTER_NORMAL:
    case Cmd::CMD_ESTOP:
    case Cmd::CMD_CLEAR_ESTOP:
    case Cmd::CMD_ENTER_SYSID:
      return true;
    case Cmd::CMD_ENTER_JOINT_IMPEDANCE:
    case Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE:
    case Cmd::CMD_ENTER_TEACH:
    case Cmd::CMD_UPDATE_COMPLIANCE_SETTINGS:
      return valid_compliance_settings(req.compliance);
    case Cmd::CMD_SET_PAYLOAD:
      return valid_payload_params(req.payload_mass, req.payload_com_tcp);
    default:
      return false;
  }
}

}  // namespace lite6_interfaces
