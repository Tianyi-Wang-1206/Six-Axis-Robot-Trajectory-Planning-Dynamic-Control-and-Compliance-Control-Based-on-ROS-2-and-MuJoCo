# Observational MuJoCo snapshot, in joint1..joint6 order. No control commands.
# stamp maps the physics step boundary into ROS time; read_stamp is read() time.
builtin_interfaces/Time stamp
builtin_interfaces/Time read_stamp
float64 simulation_time
uint64 step
uint64 command_sequence
float64 control_period
float64 physics_lag
float64 command_age
float64[6] position
float64[6] velocity
float64[6] effort
