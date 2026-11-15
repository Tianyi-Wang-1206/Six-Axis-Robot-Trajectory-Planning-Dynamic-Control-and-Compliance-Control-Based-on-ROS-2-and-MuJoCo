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

# Per-axis translational damping [N*s/m] in LOCAL_WORLD_ALIGNED frame, range [1.0, 100.0].
float64[3] translation_damping

# Per-axis rotational stiffness [Nm/rad] in LOCAL_WORLD_ALIGNED frame, range [0.0, 50.0].
float64[3] rotation_stiffness

# Per-axis rotational damping [Nm*s/rad] in LOCAL_WORLD_ALIGNED frame, range [0.1, 20.0].
float64[3] rotation_damping

# Joint-space null damping [Nm*s/rad] relative to reference joint velocity, range [0.0, 20.0].
float64 nullspace_damping
