# Observation of the CTC controller's ACTUAL internal mode.
# Published by CTC, never synthesized from the last command sent to CTC.
# The supervisor checks receipt age using a monotonic clock.
builtin_interfaces/Time stamp

uint8 MODE_UNKNOWN=0
uint8 MODE_NORMAL=1
uint8 MODE_ESTOP=2
uint8 MODE_SYSID=3

uint8 MODE_JOINT_IMPEDANCE=4
uint8 MODE_CARTESIAN_IMPEDANCE=5
uint8 MODE_TEACH=6
uint8 MODE_COMPLIANCE_STOPPING=7
uint8 MODE_CARTESIAN_IMPEDANCE_TRAJECTORY=8
uint8 mode

# Actual first control-cycle timestamp of the most recent trajectory.
# A zero timestamp means no trajectory has started. Correlate with the Action UUID.
uint8[16] trajectory_id
builtin_interfaces/Time trajectory_start

# Latched cause when mode == MODE_ESTOP (reset when leaving ESTOP).
uint8 ESTOP_NONE=0
uint8 ESTOP_EXTERNAL=1
uint8 ESTOP_COMPLIANCE_POSITION_LIMIT=2
uint8 ESTOP_COMPLIANCE_VELOCITY_LIMIT=3
uint8 ESTOP_COMPLIANCE_SATURATION=4
uint8 ESTOP_TRAJECTORY_PATH_POSITION=5
uint8 ESTOP_TRAJECTORY_PATH_VELOCITY=6
uint8 ESTOP_TRAJECTORY_GOAL_TIMEOUT=7
uint8 ESTOP_TRAJECTORY_SATURATION=8
uint8 ESTOP_TRAJECTORY_MODE_CHANGED=9
uint8 ESTOP_INVALID_STATE=10
uint8 ESTOP_INVALID_COMMAND=11
uint8 ESTOP_COMMAND_WRITE=12
uint8 ESTOP_CONTROL_EXCEPTION=13
uint8 ESTOP_IMPEDANCE_TRAJECTORY_DEVIATION=14
uint8 ESTOP_IMPEDANCE_TRAJECTORY_GOAL_TIMEOUT=15
uint8 ESTOP_COMPLIANCE_STOPPING_TIMEOUT=17
uint8 estop_reason
# Zero-based joint index when applicable, or -1 if not joint-specific.
int8 estop_joint
float64 estop_value
float64 estop_limit


# Edge-triggered ControllerCommand ack values (mirrors ControllerCommand.Response.ACK_*).
uint8 ACK_NONE=0
uint8 ACK_APPLIED=1
uint8 ACK_REJECTED_STATE=2
uint8 ACK_REJECTED_SPEED=3
uint8 ACK_REJECTED_LIMIT=4
uint8 ACK_REJECTED_BUSY=5
uint8 ACK_REJECTED_EPOCH=6
uint8 ACK_REJECTED_INVALID_ARG=7

# Monotonic publish sequence, controller activation epoch, and mode-transition counter.
uint64 status_seq
uint64 controller_epoch
uint64 mode_epoch

# Most recent ControllerCommand processed inside the 1 kHz RT loop.
uint64 last_cmd_seq
uint8 last_cmd_code
uint8 last_cmd_result

# True while a rigid FollowJointTrajectory goal retains RT trajectory ownership (including stop ramp).
bool rigid_trajectory_active

# Currently active known rigid payload compensation parameters.
float64 payload_mass
float64[3] payload_com_tcp
