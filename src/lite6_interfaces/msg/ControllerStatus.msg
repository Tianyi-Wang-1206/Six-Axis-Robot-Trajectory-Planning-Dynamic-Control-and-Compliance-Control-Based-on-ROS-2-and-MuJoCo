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
uint8 mode

# Actual first control-cycle timestamp of the most recent trajectory.
# A zero timestamp means no trajectory has started. Correlate with the Action UUID.
uint8[16] trajectory_id
builtin_interfaces/Time trajectory_start
