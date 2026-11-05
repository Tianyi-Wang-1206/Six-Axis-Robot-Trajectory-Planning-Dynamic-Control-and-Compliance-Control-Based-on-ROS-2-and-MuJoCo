# Observation of the CTC controller's ACTUAL internal mode.
# Published by CTC, never synthesized from the last command sent to CTC.
# The supervisor checks receipt age using a monotonic clock.
builtin_interfaces/Time stamp

uint8 MODE_UNKNOWN=0
uint8 MODE_NORMAL=1
uint8 MODE_ESTOP=2
uint8 MODE_SYSID=3

uint8 mode
