# Sole authoritative high-level system state (published by Supervisor).
# revision increases on discrete state transitions, not on each heartbeat.
builtin_interfaces/Time stamp
uint64 revision

# Nonzero session ID for the current SysID or recovery operation; zero otherwise.
# Generate a fresh, hard-to-reuse ID at the start of each operation.
uint64 session_id

# Orthogonal operating mode and motion task phase.
uint8 MODE_INITIALIZING=0
uint8 MODE_NORMAL=1
uint8 MODE_SYSID=2
uint8 MODE_ESTOP=3
uint8 MODE_RECOVERING=4
uint8 MODE_SOFT_FAULT=5
uint8 MODE_FAULT=6
uint8 MODE_JOINT_IMPEDANCE=7
uint8 MODE_CARTESIAN_IMPEDANCE=8
uint8 MODE_TEACH=9
uint8 MODE_COMPLIANCE_SWITCHING=10
uint8 mode

uint8 MOTION_IDLE=0
uint8 MOTION_PLANNING=1
# Waiting for direct CTC acceptance.
uint8 MOTION_WAITING_CTC=2
uint8 MOTION_EXECUTING=3
uint8 MOTION_CANCELING=4
uint8 motion_phase

# Only meaningful when mode == MODE_SYSID.
uint8 SYSID_NONE=0
uint8 SYSID_PRE_ALIGN=1
uint8 SYSID_WAIT_CONTROLLER_MODE=2
uint8 SYSID_READY_FOR_EXCITATION=3
uint8 SYSID_EXCITING=4
uint8 SYSID_CALCULATING=5
uint8 SYSID_POST_ALIGN=6
uint8 SYSID_WAIT_NORMAL_MODE=7
uint8 SYSID_RETURN_ALIGN=8
uint8 SYSID_COMPLETING=9
# Planner-side validation of the received trajectory, before CTC dispatch.
uint8 SYSID_VALIDATING=10
uint8 sysid_phase

# No phase entry action is permitted until continuous physical settling is confirmed.
bool waiting_for_settle
# CTC-authored trajectory origin, correlated to this session's excitation Action.
builtin_interfaces/Time excitation_start

# Only meaningful when mode == MODE_RECOVERING.
uint8 RECOVERY_NONE=0
uint8 RECOVERY_SYNC_REFERENCE=1
uint8 RECOVERY_WAIT_NORMAL_MODE=2
uint8 RECOVERY_SETTLING=3
uint8 recovery_phase

# ControllerStatus.mode; UNKNOWN when unobserved or stale.
uint8 controller_mode
bool controller_fresh
bool joint_states_fresh
bool motion_allowed
bool estop_latched

# Informational diagnostics, not authoritative transition triggers.
int32 ERROR_SYSID_ESTIMATION=-14
int32 last_error_code
string detail
