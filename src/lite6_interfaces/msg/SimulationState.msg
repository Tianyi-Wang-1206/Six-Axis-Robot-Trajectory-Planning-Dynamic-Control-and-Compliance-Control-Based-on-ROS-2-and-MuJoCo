# Actual physics snapshot, joint1..joint6. Display only; never fed back to control.
float64[6] position
float64[6] velocity
float64 simulation_time
bool active
float64[2] gripper_position
float64[2] gripper_velocity

# Scene free-body poses: cylinder then box; each is xyz + quaternion wxyz.
# Display only; these objects are not in the planner collision model.
float64[14] object_qpos
