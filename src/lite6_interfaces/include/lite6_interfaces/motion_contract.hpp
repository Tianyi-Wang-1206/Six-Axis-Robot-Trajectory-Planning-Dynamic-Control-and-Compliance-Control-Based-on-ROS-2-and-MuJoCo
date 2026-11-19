#ifndef LITE6_INTERFACES__MOTION_CONTRACT_HPP_
#define LITE6_INTERFACES__MOTION_CONTRACT_HPP_

#include <cstdint>

namespace lite6_interfaces::contract
{

// Primary 1 kHz control and sampling cadence.
inline constexpr double kControlRateHz = 1000.0;
inline constexpr double kControlDtSec = 0.001;

// Nested speed and velocity tolerance hierarchy:
//   kPlannerSettleSpeedRadPerSec < kControllerEntryMaxSpeedRadPerSec <= kClearEstopMaxSpeedRadPerSec
// Planners confirm settling strictly below controller admission thresholds so
// 100 Hz SHM -> 1 kHz RT jitter never induces spurious speed rejections.
inline constexpr double kPlannerSettleSpeedRadPerSec = 0.015;
inline constexpr double kRigidGoalVelocityToleranceRadPerSec = 0.02;
inline constexpr double kControllerEntryMaxSpeedRadPerSec = 0.03;
inline constexpr double kClearEstopMaxSpeedRadPerSec = 0.05;

// Bounded wait and freshness timeouts across RT controller and supervisors.
inline constexpr double kComplianceStoppingTimeoutSec = 1.5;
inline constexpr int64_t kControllerCmdWaitTimeoutMs = 100;
inline constexpr double kControllerStatusStaleTimeoutSec = 0.25;
inline constexpr int64_t kControllerStatusStaleTimeoutNs = 250'000'000LL;
inline constexpr double kJointStateStaleTimeoutSec = 0.10;
inline constexpr int64_t kJointStateStaleTimeoutNs = 100'000'000LL;

// Validated compliance and impedance gain bounds.
inline constexpr double kMinJointStiffness = 0.0;
inline constexpr double kMaxJointStiffness = 100.0;
inline constexpr double kMinJointDamping = 0.1;
inline constexpr double kMaxJointDamping = 20.0;

inline constexpr double kMinTranslationStiffness = 0.0;
inline constexpr double kMaxTranslationStiffness = 1000.0;
inline constexpr double kMinTranslationDamping = 0.1;
inline constexpr double kMaxTranslationDamping = 2.0;

inline constexpr double kMinRotationStiffness = 0.0;
inline constexpr double kMaxRotationStiffness = 50.0;
inline constexpr double kMinRotationDamping = 0.1;
inline constexpr double kMaxRotationDamping = 2.0;

inline constexpr double kMinComplianceSensitivity = 1.0;
inline constexpr double kMaxComplianceSensitivity = 5.0;

inline constexpr double kMinNullspaceStiffness = 0.0;
inline constexpr double kMaxNullspaceStiffness = 50.0;
inline constexpr double kMinNullspaceDamping = 0.0;
inline constexpr double kMaxNullspaceDamping = 20.0;

// Feedforward wrench bounds for LOCAL_WORLD_ALIGNED TCP force/torque commands.
inline constexpr double kMaxFeedforwardForceN = 30.0;
inline constexpr double kMaxFeedforwardTorqueNm = 5.0;

// Impedance trajectory entry blend duration bounds.
inline constexpr double kMinTransitionTimeSec = 0.0;
inline constexpr double kMaxTransitionTimeSec = 5.0;
inline constexpr double kDefaultImpedanceTransitionTimeSec = 0.3;

// Known rigid payload bounds (TCP-frame mass & CoM offset).
inline constexpr double kMinPayloadMassKg = 0.0;
inline constexpr double kMaxPayloadMassKg = 2.0;
inline constexpr double kMaxPayloadComRadiusM = 0.30;

}  // namespace lite6_interfaces::contract

#endif  // LITE6_INTERFACES__MOTION_CONTRACT_HPP_
