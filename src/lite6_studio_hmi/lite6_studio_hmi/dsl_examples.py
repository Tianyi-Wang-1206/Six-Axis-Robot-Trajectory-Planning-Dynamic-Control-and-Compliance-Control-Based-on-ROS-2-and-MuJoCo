"""Built-in trajectory script example programs for the Robot Trajectory IDE."""

PRESET_PROGRAMS = {
    "1. Chaotic 10-Point Cartesian PVA (Full Workspace)": """# 10-point full-workspace Cartesian (P, V, A) C3 Septic Hermite demo.
# Waypoints 1-9 whip across front, left, behind (-X), right, and high overhead (z up to 0.63 m)
# with varied chaotic 6-DOF poses, twists, and accelerations; waypoint 10 brings the robot smoothly to rest.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=1.0, acc=1.0)

MovePVA(space=CARTESIAN, vel=1.0, acc=1.0) {
  point(
    p=Pose(xyz=[ 0.1729,  0.2923,  0.5212], rpy=[-2.5877, -1.1256,  0.7407]),
    v=[-0.125,  0.088,  0.018,  0.016, -0.077,  0.357],
    a=[ 0.096, -0.019, -0.270, -0.627,  0.651, -0.452]
  )
  point(
    p=Pose(xyz=[-0.0818,  0.2619,  0.1772], rpy=[ 2.7891,  0.0936,  1.9541]),
    v=[-0.085, -0.049, -0.010, -0.087, -0.103,  0.468],
    a=[ 0.035,  0.101,  0.043,  0.319,  0.528,  0.050]
  )
  point(
    p=Pose(xyz=[-0.2934, -0.0279,  0.5823], rpy=[-2.4233, -1.3333,  2.7146]),
    v=[-0.055, -0.052, -0.077,  0.110, -0.201,  0.195],
    a=[-0.136, -0.028, -0.292,  0.067, -0.865, -0.006]
  )
  point(
    p=Pose(xyz=[-0.1894, -0.2092,  0.1770], rpy=[ 2.6821,  0.0435, -2.2083]),
    v=[ 0.008, -0.009,  0.037, -0.145,  0.164, -0.058],
    a=[-0.045, -0.179,  0.201, -1.253, -0.065, -0.113]
  )
  point(
    p=Pose(xyz=[-0.2256, -0.0067,  0.6276], rpy=[-2.0318, -1.4591,  2.1900]),
    v=[ 0.035,  0.126,  0.052, -0.046,  0.189, -0.608],
    a=[ 0.011, -0.044, -0.074,  0.366, -0.194, -0.143]
  )
  point(
    p=Pose(xyz=[ 0.0620,  0.2693,  0.1736], rpy=[ 2.7415,  0.1139,  1.5819]),
    v=[ 0.098, -0.063, -0.005, -0.169, -0.107, -0.340],
    a=[-0.203,  0.057,  0.150,  0.473,  0.524,  0.197]
  )
  point(
    p=Pose(xyz=[ 0.2180,  0.1096,  0.6175], rpy=[-2.4493, -1.4760, -0.0711]),
    v=[-0.043, -0.089,  0.067,  0.001, -0.274, -0.329],
    a=[ 0.230,  0.077, -0.308, -0.281,  1.314, -0.093]
  )
  point(
    p=Pose(xyz=[ 0.0873, -0.2605,  0.1795], rpy=[ 2.7538,  0.1271, -1.0711]),
    v=[-0.142, -0.106,  0.047, -0.261, -0.176, -0.492],
    a=[-0.055, -0.092,  0.087, -0.614, -0.415, -0.080]
  )
  point(
    p=Pose(xyz=[-0.1371, -0.3031,  0.5162], rpy=[-2.8316, -1.1182, -2.0895]),
    v=[ 0.133, -0.051,  0.071, -0.328, -0.181,  0.382],
    a=[ 0.009, -0.085, -0.305,  0.986, -0.200,  0.223]
  )
  point(
    p=Pose(xyz=[ 0.2757,  0.0000,  0.1594], rpy=[ 3.14159, 0.0,    0.0]),
    v=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    a=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  )
}
""",
    "2. Chaotic 10-Point Joint PVA (Full Workspace)": """# 10-point full-workspace high-speed Joint-space (P, V, A) 3rd-order time-optimal demo.
# Waypoints 1-9 whip rapidly across all 6 joints with varied chaotic positions, velocities,
# and accelerations (Full Ruckig with position limits); waypoint 10 brings the robot smoothly to rest.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=1.0, acc=1.0)

MovePVA(space=JOINT, vel=1.0, acc=1.0) {
  point(
    p=[-2.870,  0.199,  1.355,  2.451, -0.726, -2.723],
    v=[ 1.554, -0.595,  1.575, -1.983, -1.093,  1.815],
    a=[ 1.225, -0.875, -0.883,  3.362,  4.328, -1.762]
  )
  point(
    p=[ 1.671, -0.096,  1.968, -1.569,  0.139,  2.828],
    v=[ 1.507, -0.712, -0.948,  1.264,  1.321,  1.379],
    a=[ 1.842, -0.922,  1.261, -4.436, -2.800, -3.500]
  )
  point(
    p=[-2.111, -0.575,  0.811,  1.892,  0.666,  2.968],
    v=[-1.638,  1.125, -0.704,  1.745, -1.592, -1.716],
    a=[ 1.581,  0.408,  2.165, -3.090,  2.800,  2.213]
  )
  point(
    p=[ 0.070,  0.051,  1.609, -1.359, -0.356,  1.919],
    v=[-1.550,  0.773,  1.271,  1.254, -2.051, -1.508],
    a=[-1.317, -0.920,  1.472, -2.800,  3.111, -2.872]
  )
  point(
    p=[ 2.650, -0.139,  1.761,  2.642,  0.376, -0.017],
    v=[ 1.302,  0.731, -1.093,  2.132,  1.563,  2.208],
    a=[-1.225,  0.875,  1.618,  2.026,  2.668, -1.643]
  )
  point(
    p=[-0.528,  0.554,  1.303,  0.925, -0.222,  1.997],
    v=[-2.046,  0.895,  1.159, -2.046, -1.662, -2.277],
    a=[-1.225,  1.051,  1.400,  2.514, -1.613,  2.074]
  )
  point(
    p=[-2.846, -0.135,  1.892, -0.877, -0.182, -1.973],
    v=[ 1.499, -0.733, -0.782, -2.315,  1.659, -1.510],
    a=[ 1.225,  0.875, -2.134, -4.358, -3.516, -3.500]
  )
  point(
    p=[ 0.233, -0.456,  1.089, -2.339,  0.696, -0.285],
    v=[-2.135,  0.901, -0.526, -2.133,  1.573,  1.557],
    a=[-0.901,  1.022,  1.400, -4.596, -1.851,  4.425]
  )
  point(
    p=[-2.769,  0.017,  0.950,  2.441,  0.033, -0.617],
    v=[ 1.707,  0.929, -0.444,  1.629, -1.777,  1.913],
    a=[ 1.524,  0.552, -1.589, -2.749,  1.885, -3.500]
  )
  point(
    p=[ 0.000,  0.350,  1.000,  0.000,  0.650,  0.000],
    v=[ 0.000,  0.000,  0.000,  0.000,  0.000,  0.000],
    a=[ 0.000,  0.000,  0.000,  0.000,  0.000,  0.000]
  )
}
""",
    "3. Horizontal Cartesian Path Blending (Stop-and-Go vs Smooth)": """# Large horizontal-plane (z = 0.250 m) Cartesian trajectory splicing comparison
# in robot base frame (+Y is left, -Y is right):
# 1. Leave ZERO and move to READY on the z = 0.250 m plane ([0.260, 0.000, 0.250]).
# 2. Loop 1 (no blending): wide horizontal zigzag MoveL left (+Y), wide MoveC arc
#    across to the right (-Y), and zigzag MoveL back to READY, stopping at every corner.
# 3. Loop 2 (smooth C3 corner blending): exact same horizontal route in one continuous motion.
joint READY = [0.0, 0.1066, 1.1100, 0.0, 1.0034, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=1.0, acc=1.0)

# --- Loop 1: Exact stop-and-go at every corner on z = 0.250 m plane (blend = 0) ---
MoveL(Pose(xyz=[0.165,  0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.335,  0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.165,  0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.240,  0.275, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveC(
  via=[0.370, 0.000, 0.250],
  target=Pose(xyz=[0.240, -0.275, 0.250], rpy=[3.14159, 0.0, 0.0]),
  vel=1.0, acc=1.0
)
MoveL(Pose(xyz=[0.165, -0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.335, -0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.165, -0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveL(Pose(xyz=[0.260,  0.000, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)

Wait(duration=0.10)

# --- Loop 2: Same horizontal route with smooth C3 corner blending ---
MoveL(Pose(xyz=[0.165,  0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.335,  0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.165,  0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.240,  0.275, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveC(
  via=[0.370, 0.000, 0.250],
  target=Pose(xyz=[0.240, -0.275, 0.250], rpy=[3.14159, 0.0, 0.0]),
  vel=1.0, acc=1.0, blend=0.025
)
MoveL(Pose(xyz=[0.165, -0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.335, -0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.165, -0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.260,  0.000, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
""",
    "4. Compliant Stack & Side-Grasp Arc (300 g Objects)": """# Full physical manipulation demo matching scene.xml object positions:
# - Cylinder at [0.24, -0.15, 0.05] (height 100 mm, dia 30 mm, mass 0.30 kg)
# - Box at [0.24, 0.12, 0.04] (30 x 40 x 80 mm, mass 0.30 kg)
joint ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Exit wrist singularity (joint5=0 at ZERO) before running Cartesian MoveL
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.55, acc=0.55)
Gripper(action=OPEN, wait=true)

# 1. MoveL directly above cylinder and stop
MoveL(Pose(xyz=[0.240, -0.150, 0.160], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)

# 2. Descend to z=85 mm and close gripper to pick 0.30 kg cylinder
MoveL(Pose(xyz=[0.240, -0.150, 0.085], rpy=[3.14159, 0.0, 0.0]), vel=0.8, acc=0.7)
Gripper(action=CLOSE, wait=true)
SetPayload(mass=0.30, com=[0.0, 0.0, 0.0])
Wait(duration=0.05)

# 3. Lift to z=200 mm and smoothly blend into a long MoveC arc above the box
MoveL(Pose(xyz=[0.240, -0.150, 0.200], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveC(
  via=Pose(xyz=[0.365, -0.015, 0.220], rpy=[3.14159, 0.0, 0.0]),
  target=Pose(xyz=[0.240, 0.120, 0.200], rpy=[3.14159, 0.0, 0.0]),
  vel=0.9, acc=0.9
)

# 4. Cartesian impedance descent: cylinder contacts box top (~z=0.165),
#    while equilibrium descends to z=0.135 to clearly demonstrate compliant yielding
ImpedanceCartesian(kt=[450, 450, 100], dt=[0.85, 0.85, 0.80], kr=[25, 25, 18], dr=[0.85, 0.85, 0.80]) {
  MoveL(Pose(xyz=[0.240, 0.120, 0.135], rpy=[3.14159, 0.0, 0.0]), vel=0.18, acc=0.20)
}

# 5. Stop, open gripper, reset payload, lift to z=200 mm, and blend MoveC to approach box from -Y side
Wait(duration=0.1)
Gripper(action=OPEN, wait=true)
SetPayload(mass=0.0)
Wait(duration=0.1)
MoveL(Pose(xyz=[0.240, 0.120, 0.200], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0, blend=0.025)
MoveC(
  via=Pose(xyz=[0.240, 0.055, 0.140], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, 0.045, 0.055], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.6, acc=0.6,
  blend=0.025
)

# Approach from the side at z=55 mm without touching the stacked cylinder,
# then close gripper across the 30 mm X-width (0.30 kg box + 0.30 kg stacked cylinder = 0.60 kg)
MoveL(Pose(xyz=[0.240, 0.112, 0.055], rpy=[3.14159, 1.309, -1.5708]), vel=0.5, acc=0.5)
Gripper(action=CLOSE, wait=true)
SetPayload(mass=0.60, com=[0.0, 0.0, 0.0])
Wait(duration=0.1)

# 6. Large vertical MoveC arc (rise up, then descend to ~3.5 cm above tabletop)
MoveC(
  via=Pose(xyz=[0.250, 0.020, 0.215], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, -0.070, 0.090], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.8, acc=0.8
)

# 7. Compliant Cartesian impedance placement onto table, open gripper, reset payload, retract clear, MoveJ ZERO
ImpedanceCartesian(kt=[450, 450, 120], dt=[0.85, 0.85, 0.80], kr=[25, 25, 18], dr=[0.85, 0.85, 0.80]) {
  MoveL(Pose(xyz=[0.240, -0.070, 0.038], rpy=[3.14159, 1.309, -1.5708]), vel=0.18, acc=0.20)
}
Gripper(action=OPEN, wait=true)
SetPayload(mass=0.0)
Wait(duration=0.1)
MoveL(Pose(xyz=[0.235, -0.118, 0.090], rpy=[3.14159, 1.309, -1.5708]), vel=1.0, acc=1.0, blend=0.025)
MoveL(Pose(xyz=[0.200, -0.150, 0.240], rpy=[3.14159, 0.0, 0.0]), vel=1.0, acc=1.0)
MoveJ(ZERO, vel=1.0, acc=1.0)
""",
}
