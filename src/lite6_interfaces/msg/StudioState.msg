# Authoritative industrial state machine and execution telemetry published by lite6_studio_planner.
builtin_interfaces/Time stamp
uint64 revision

uint8 STATE_BOOT_INIT=0
uint8 STATE_IDLE=1
uint8 STATE_COMPILED_READY=2
uint8 STATE_RUNNING=3
uint8 STATE_PAUSED=4
uint8 STATE_STEPPING=5
uint8 STATE_STOPPING=6
uint8 STATE_ESTOP=7
uint8 STATE_FAULT=8
uint8 STATE_RECOVERING=9
uint8 state

string loaded_program_id
int32 current_line_number
int32 current_statement_index
int32 current_block_index
int32 total_blocks
float64 block_progress_percent
float64 total_progress_percent

# Observed low-level controller and gripper states.
uint8 controller_mode
bool controller_fresh
bool joint_states_fresh
uint8 gripper_state
bool gripper_ready
bool estop_latched
uint8 estop_reason
int32 last_error_code

# Live Cartesian tracking deflection during impedance trajectory execution (zero otherwise).
float64[3] cartesian_pos_error_m
float64[3] cartesian_rot_error_rad

# Sensorless Generalized Momentum Observer (GMO) external load estimates.
float64[6] external_joint_torque
float64[6] external_cartesian_wrench
bool contact_detected

string detail
