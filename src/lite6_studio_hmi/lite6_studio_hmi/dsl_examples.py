"""Built-in trajectory script example programs for the Robot Trajectory IDE."""

PRESET_PROGRAMS = {
    "1. Chaotic 10-Point Cartesian PVA (Full Workspace)": """# 10-point full-workspace Cartesian (P, V, A) C3 Septic Hermite demo.
# Waypoints 1-9 whip across front, left, behind (-X), right, and high overhead (z up to 0.63 m)
# with varied chaotic 6-DOF poses, twists, and accelerations; waypoint 10 brings the robot smoothly to rest.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.60, acc=0.60)

MovePVA(space=CARTESIAN, vel=0.90, acc=0.90) {
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
    "2. Horizontal Cartesian Path Blending (Stop-and-Go vs Smooth)": """# Large horizontal-plane (z = 0.250 m) Cartesian trajectory splicing comparison
# in robot base frame (+Y is left, -Y is right):
# 1. Leave ZERO and move to READY on the z = 0.250 m plane ([0.260, 0.000, 0.250]).
# 2. Loop 1 (no blending): wide horizontal zigzag MoveL left (+Y), wide MoveC arc
#    across to the right (-Y), and zigzag MoveL back to READY, stopping at every corner.
# 3. Loop 2 (smooth C3 corner blending): exact same horizontal route in one continuous motion.
joint READY = [0.0, 0.1066, 1.1100, 0.0, 1.0034, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.60, acc=0.60)

# --- Loop 1: Exact stop-and-go at every corner on z = 0.250 m plane (blend = 0) ---
MoveL(Pose(xyz=[0.165,  0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.335,  0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.165,  0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.240,  0.275, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveC(
  via=[0.370, 0.000, 0.250],
  target=Pose(xyz=[0.240, -0.275, 0.250], rpy=[3.14159, 0.0, 0.0]),
  vel=0.65, acc=0.65
)
MoveL(Pose(xyz=[0.165, -0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.335, -0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.165, -0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
MoveL(Pose(xyz=[0.260,  0.000, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)

Wait(duration=0.50)

# --- Loop 2: Same horizontal route with smooth C3 corner blending ---
MoveL(Pose(xyz=[0.165,  0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.335,  0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.165,  0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.240,  0.275, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveC(
  via=[0.370, 0.000, 0.250],
  target=Pose(xyz=[0.240, -0.275, 0.250], rpy=[3.14159, 0.0, 0.0]),
  vel=0.65, acc=0.65, blend=0.025
)
MoveL(Pose(xyz=[0.165, -0.200, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.335, -0.135, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.165, -0.070, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65, blend=0.025)
MoveL(Pose(xyz=[0.260,  0.000, 0.250], rpy=[3.14159, 0.0, 0.0]), vel=0.65, acc=0.65)
""",
    "3. Compliant Stack & Side-Grasp Arc (300 g Objects)": """# Full physical manipulation demo matching scene.xml object positions:
# - Cylinder at [0.24, -0.15, 0.05] (height 100 mm, dia 30 mm, mass 0.30 kg)
# - Box at [0.24, 0.12, 0.04] (30 x 40 x 80 mm, mass 0.30 kg)
joint ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Exit wrist singularity (joint5=0 at ZERO) before running Cartesian MoveL
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.55, acc=0.55)
Gripper(action=OPEN, wait=true)

# 1. MoveL directly above cylinder and stop
MoveL(Pose(xyz=[0.240, -0.150, 0.160], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45)

# 2. Descend to z=85 mm and close gripper to pick 0.30 kg cylinder
MoveL(Pose(xyz=[0.240, -0.150, 0.085], rpy=[3.14159, 0.0, 0.0]), vel=0.28, acc=0.30)
Gripper(action=CLOSE, wait=true)
SetPayload(mass=0.30, com=[0.0, 0.0, 0.0])
Wait(duration=0.20)

# 3. Lift to z=200 mm and smoothly blend into a long MoveC arc above the box
MoveL(Pose(xyz=[0.240, -0.150, 0.200], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45, blend=0.025)
MoveC(
  via=Pose(xyz=[0.365, -0.015, 0.220], rpy=[3.14159, 0.0, 0.0]),
  target=Pose(xyz=[0.240, 0.120, 0.200], rpy=[3.14159, 0.0, 0.0]),
  vel=0.45, acc=0.45
)

# 4. Cartesian impedance descent: cylinder contacts box top (~z=0.165),
#    while equilibrium descends to z=0.135 to clearly demonstrate compliant yielding
ImpedanceCartesian(kt=[450, 450, 100], dt=[0.85, 0.85, 0.80], kr=[25, 25, 18], dr=[0.85, 0.85, 0.80], null_damping=0.15) {
  MoveL(Pose(xyz=[0.240, 0.120, 0.135], rpy=[3.14159, 0.0, 0.0]), vel=0.18, acc=0.20)
}

# 5. Stop, open gripper, reset payload, lift to z=200 mm, and blend MoveC to approach box from -Y side
Wait(duration=0.30)
Gripper(action=OPEN, wait=true)
SetPayload(mass=0.0)
Wait(duration=0.20)
MoveL(Pose(xyz=[0.240, 0.120, 0.200], rpy=[3.14159, 0.0, 0.0]), vel=0.40, acc=0.40, blend=0.025)
MoveC(
  via=Pose(xyz=[0.240, 0.055, 0.140], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, 0.045, 0.055], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.35, acc=0.35
)

# Approach from the side at z=55 mm without touching the stacked cylinder,
# then close gripper across the 30 mm X-width (0.30 kg box + 0.30 kg stacked cylinder = 0.60 kg)
MoveL(Pose(xyz=[0.240, 0.112, 0.055], rpy=[3.14159, 1.309, -1.5708]), vel=0.22, acc=0.25)
Gripper(action=CLOSE, wait=true)
SetPayload(mass=0.60, com=[0.0, 0.0, 0.0])
Wait(duration=0.25)

# 6. Large vertical MoveC arc (rise up, then descend to ~3.5 cm above tabletop)
MoveC(
  via=Pose(xyz=[0.250, 0.020, 0.215], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, -0.070, 0.090], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.40, acc=0.40
)

# 7. Compliant Cartesian impedance placement onto table, open gripper, reset payload, retract clear, MoveJ ZERO
ImpedanceCartesian(kt=[450, 450, 120], dt=[0.85, 0.85, 0.80], kr=[25, 25, 18], dr=[0.85, 0.85, 0.80], null_damping=0.15) {
  MoveL(Pose(xyz=[0.240, -0.070, 0.038], rpy=[3.14159, 1.309, -1.5708]), vel=0.18, acc=0.20)
}
Gripper(action=OPEN, wait=true)
SetPayload(mass=0.0)
Wait(duration=0.20)
MoveL(Pose(xyz=[0.235, -0.118, 0.090], rpy=[3.14159, 1.309, -1.5708]), vel=0.35, acc=0.35)
MoveL(Pose(xyz=[0.200, -0.150, 0.240], rpy=[3.14159, 0.0, 0.0]), vel=0.35, acc=0.35)
MoveJ(ZERO, vel=0.50, acc=0.50)
""",
}
