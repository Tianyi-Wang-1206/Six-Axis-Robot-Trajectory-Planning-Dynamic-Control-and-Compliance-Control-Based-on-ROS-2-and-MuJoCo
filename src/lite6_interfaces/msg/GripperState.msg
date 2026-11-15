# Independent gripper observation, not gated by the arm Supervisor.
# GRASPED is a simulated contact estimate, not a hardware gripper sensor signal.
builtin_interfaces/Time stamp
uint8 BLOCKED=0
uint8 STOPPED=1
uint8 OPENING=2
uint8 CLOSING=3
uint8 OPEN=4
uint8 CLOSED=5
uint8 GRASPED=6
uint8 FAULT=7
uint8 state
bool ready
bool active
uint64 applied_sequence
float64[2] position
float64[2] velocity
float64 effort
float64 stiffness
float64 damping
float64 max_force
float64 max_velocity
string detail
