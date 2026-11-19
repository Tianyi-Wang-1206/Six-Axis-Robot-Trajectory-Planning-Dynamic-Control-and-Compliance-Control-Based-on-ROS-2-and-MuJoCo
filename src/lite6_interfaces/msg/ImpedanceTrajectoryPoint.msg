# Time-varying equilibrium reference sample for 1 kHz Cartesian impedance trajectory tracking.
builtin_interfaces/Duration time_from_start

# Reference joint configuration, velocity, and acceleration [rad, rad/s, rad/s^2].
# Used by Lite6CTCController to compute exact time-varying equilibrium pose T_d(t) = FK(q_d(t)),
# reference twist v_d(t) = J(q_d(t)) * dq_d(t), and null-space posture damping reference.
float64[6] positions
float64[6] velocities
float64[6] accelerations

# Per-axis translational stiffness [N/m] in LOCAL_WORLD_ALIGNED frame, range [0.0, 1000.0].
float64[3] translation_stiffness

# Per-axis translational damping ratio zeta_t [-] in LOCAL_WORLD_ALIGNED frame, range [0.1, 2.0].
float64[3] translation_damping

# Per-axis rotational stiffness [Nm/rad] in LOCAL_WORLD_ALIGNED frame, range [0.0, 50.0].
float64[3] rotation_stiffness

# Per-axis rotational damping ratio zeta_r [-] in LOCAL_WORLD_ALIGNED frame, range [0.1, 2.0].
float64[3] rotation_damping

# Joint-space null damping [Nm*s/rad] relative to reference joint velocity, range [0.0, 20.0].
float64 nullspace_damping

# Optional joint-space null stiffness [Nm/rad] relative to reference configuration, range [0.0, 50.0].
float64 nullspace_stiffness 0.0

# Optional Cartesian feedforward wrench [fx, fy, fz, tx, ty, tz] in LOCAL_WORLD_ALIGNED TCP frame [N, Nm].
float64[6] wrench_ff
