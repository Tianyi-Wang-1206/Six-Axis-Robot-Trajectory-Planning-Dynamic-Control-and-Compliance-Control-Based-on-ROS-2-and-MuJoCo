"""Built-in RobotScript example programs for Robot Studio HMI."""

PRESET_PROGRAMS = {
    "1. Chaotic 10-Point Cartesian PVA (MovePVA)": """# 10-point Cartesian (P, V, A) C3 Septic Hermite interpolation demo.
# Waypoints 1-9 specify varied chaotic 6-DOF poses, twists, and accelerations;
# waypoint 10 brings the robot smoothly to rest at the ready pose.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.55, acc=0.55)

MovePVA(space=CARTESIAN, vel=0.85, acc=0.85) {
  point(
    p=Pose(xyz=[0.2124,  0.1338, 0.2533], rpy=[-2.8336, -0.1187,  0.5317]),
    v=[ 0.031,  0.070,  0.003,  0.107, -0.265,  0.482],
    a=[ 0.206, -0.043, -0.125, -0.262, -0.303, -0.853]
  )
  point(
    p=Pose(xyz=[0.2587,  0.2382, 0.2156], rpy=[ 2.9115, -0.2390,  1.5524]),
    v=[-0.027,  0.003, -0.005, -0.172, -0.202,  0.349],
    a=[ 0.020, -0.129,  0.145,  0.057, -0.109, -0.288]
  )
  point(
    p=Pose(xyz=[0.2307,  0.0416, 0.2331], rpy=[ 2.8858,  0.1584,  0.9546]),
    v=[ 0.003, -0.083, -0.050, -0.226,  0.138, -0.193],
    a=[-0.037,  0.058, -0.046,  0.240,  0.158, -0.150]
  )
  point(
    p=Pose(xyz=[0.3409, -0.0639, 0.1991], rpy=[ 2.6372, -0.0843,  0.5830]),
    v=[ 0.027, -0.071,  0.063,  0.084, -0.363, -0.266],
    a=[-0.230, -0.129, -0.096,  0.087,  0.647, -0.780]
  )
  point(
    p=Pose(xyz=[0.1890, -0.1308, 0.2240], rpy=[ 2.8432, -0.0359, -0.6136]),
    v=[ 0.006, -0.016,  0.062,  0.080, -0.352, -0.816],
    a=[ 0.115, -0.048, -0.159,  0.070, -0.012,  0.572]
  )
  point(
    p=Pose(xyz=[0.2535, -0.2487, 0.2100], rpy=[-2.8741, -0.2128, -1.6644]),
    v=[ 0.028, -0.043,  0.071, -0.165, -0.290, -0.150],
    a=[-0.182,  0.200, -0.139,  0.934,  0.783,  0.266]
  )
  point(
    p=Pose(xyz=[0.2363, -0.0442, 0.2393], rpy=[-2.8578,  0.1770, -1.0515]),
    v=[ 0.022,  0.036, -0.007,  0.246, -0.066,  0.199],
    a=[ 0.206, -0.138,  0.128, -0.441, -1.141, -0.318]
  )
  point(
    p=Pose(xyz=[0.3449,  0.0443, 0.1998], rpy=[-2.6216, -0.0266, -0.7454]),
    v=[ 0.000,  0.060,  0.036,  0.005, -0.118,  0.390],
    a=[-0.116, -0.020,  0.057,  0.157,  0.256,  0.461]
  )
  point(
    p=Pose(xyz=[0.2009,  0.1058, 0.2102], rpy=[-2.8422,  0.0354,  0.3925]),
    v=[-0.035, -0.063, -0.050, -0.345, -0.002,  0.204],
    a=[ 0.248, -0.064,  0.045, -0.046, -0.545, -0.968]
  )
  point(
    p=Pose(xyz=[0.2757,  0.0000, 0.1594], rpy=[ 3.14159, 0.0,    0.0]),
    v=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    a=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  )
}
""",
    "2. Cartesian Path Blending (Stop-and-Go vs Smooth)": """# Cartesian trajectory splicing comparison in robot base frame (+Y is left, -Y is right):
# 1. Leave ZERO and move to READY joint configuration.
# 2. Loop 1 (no blending): zigzag MoveL to the left (+Y), MoveC arc across to the right (-Y),
#    and zigzag MoveL back to READY, stopping at every corner (rest-to-rest).
# 3. Loop 2 (smooth C3 corner blending): exact same geometric route in one continuous motion.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

SetPayload(mass=0.0)
MoveJ(READY, vel=0.55, acc=0.55)

# --- Loop 1: Exact stop-and-go at every corner (blend = 0) ---
MoveL(Pose(xyz=[0.220,  0.040, 0.210], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.310,  0.080, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.220,  0.120, 0.215], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.295,  0.160, 0.180], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveC(
  via=[0.345, 0.000, 0.245],
  target=Pose(xyz=[0.295, -0.160, 0.180], rpy=[3.14159, 0.0, 0.0]),
  vel=0.55, acc=0.55
)
MoveL(Pose(xyz=[0.220, -0.120, 0.215], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.310, -0.080, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.220, -0.040, 0.210], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.2757, 0.000, 0.1594], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)

Wait(duration=0.50)

# --- Loop 2: Same route with smooth C3 corner blending ---
MoveL(Pose(xyz=[0.220,  0.040, 0.210], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.310,  0.080, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.220,  0.120, 0.215], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.295,  0.160, 0.180], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveC(
  via=[0.345, 0.000, 0.245],
  target=Pose(xyz=[0.295, -0.160, 0.180], rpy=[3.14159, 0.0, 0.0]),
  vel=0.55, acc=0.55, blend=0.018
)
MoveL(Pose(xyz=[0.220, -0.120, 0.215], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.310, -0.080, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.220, -0.040, 0.210], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55, blend=0.018)
MoveL(Pose(xyz=[0.2757, 0.000, 0.1594], rpy=[3.14159, 0.0, 0.0]), vel=0.55, acc=0.55)
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

# Approach from the side without palm collision and close gripper across the 30 mm X-width (0.30 kg box)
MoveL(Pose(xyz=[0.240, 0.112, 0.055], rpy=[3.14159, 1.309, -1.5708]), vel=0.22, acc=0.25)
Gripper(action=CLOSE, wait=true)
SetPayload(mass=0.30, com=[0.0, 0.0, 0.0])
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
