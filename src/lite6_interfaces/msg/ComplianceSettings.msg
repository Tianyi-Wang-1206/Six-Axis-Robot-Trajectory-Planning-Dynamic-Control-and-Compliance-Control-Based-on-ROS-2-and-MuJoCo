# Physical gains; no measured or simulated force input is used.
float64 joint_stiffness 20.0
float64 joint_damping 3.0
float64 translation_stiffness 200.0
float64 translation_damping 25.0
float64 rotation_stiffness 8.0
float64 rotation_damping 1.5
# 1 (firm) to 5 (light): bounded friction assistance and positive damping.
float64 sensitivity 3.0
# Same-mode updates preserve the equilibrium unless explicitly requested.
bool recapture false
# Optional null-space posture regulation gains for interactive Cartesian impedance [0.0..50.0, 0.0..20.0].
float64 nullspace_stiffness 0.0
float64 nullspace_damping 0.5
