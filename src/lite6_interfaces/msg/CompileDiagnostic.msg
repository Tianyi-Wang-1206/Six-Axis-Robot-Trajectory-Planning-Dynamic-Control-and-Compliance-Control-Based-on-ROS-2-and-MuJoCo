# Single structured diagnostic produced by the 5-stage Studio compilation pipeline.
uint8 SEVERITY_ERROR=1
uint8 SEVERITY_WARNING=2
uint8 severity

uint8 STAGE_LEX_SYNTAX=1
uint8 STAGE_SEMANTIC=2
uint8 STAGE_PLANNING=3
uint8 STAGE_KINEMATIC_LIMIT=4
uint8 STAGE_COLLISION=5
uint8 stage

# 1-based line and column numbers in the user's RobotScript source code.
int32 line_number
int32 column_number

# 0-based statement/segment index (-1 if global or before statement resolution).
int32 statement_index

# Source text snippet of the offending statement or token.
string command_text

# Short machine-readable diagnostic identifier (e.g., "ERR_SYNTAX", "ERR_JOINT_VEL_LIMIT", "ERR_COLLISION").
string code

# Detailed human-readable diagnostic including exact offending values, limits, or colliding link names.
string message
