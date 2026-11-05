#ifndef LITE6_PLANNER_MOTION_TOLERANCES_HPP_
#define LITE6_PLANNER_MOTION_TOLERANCES_HPP_

namespace lite6_planner::tolerance {

// Numerical allowance for joint targets, IK and geometric path calculations [rad].
inline constexpr double kJointPosition = 1e-8;
// Hardware observations may penetrate a soft joint limit slightly [rad].
// Only the planning copy is projected to the limit; physical feedback is untouched.
inline constexpr double kObservedPosition = 1e-4;
// Treat only negligible observed velocities as zero [rad/s]. Larger velocities
// are preserved, including throughout the existing near-rest admission band.
inline constexpr double kZeroVelocity = 1e-5;
inline constexpr double kStationaryVelocity = 0.02;

// Existing sampled-trajectory validation allowances, also used for SysID.
inline constexpr double kTrajectoryPosition = 1e-6;      // rad
inline constexpr double kTrajectoryVelocity = 1e-5;      // rad/s
inline constexpr double kTrajectoryAcceleration = 1e-4;  // rad/s^2

} // namespace lite6_planner::tolerance
#endif
