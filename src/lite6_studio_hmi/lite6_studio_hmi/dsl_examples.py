"""Built-in RobotScript example programs for Robot Studio HMI."""

PRESET_PROGRAMS = {
    "1. Pick & Place + Impedance (Full Demo)": """# Complete industrial cycle demonstrating MoveJ, blended MoveL/MoveC,
# time-varying Cartesian impedance contact, and decoupled Gripper actions.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

# Move to ready joint pose (TCP ~ [0.2757, 0.0, 0.1594] m, rpy ~ [pi, 0, 0])
MoveJ(READY, vel=0.50, acc=0.50)
Gripper(action=OPEN, wait=true)

# Smooth G2/C3 corner-blended Cartesian approach
MoveL(Pose(xyz=[0.315, 0.000, 0.160], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45, blend=0.015)
MoveC(
  via=Pose(xyz=[0.315, 0.040, 0.160], rpy=[3.14159, 0.0, 0.0]),
  target=Pose(xyz=[0.275, 0.045, 0.160], rpy=[3.14159, 0.0, 0.0]),
  vel=0.45, acc=0.45
)

# Time-varying equilibrium Cartesian impedance block for compliant approach/grasp
ImpedanceCartesian(kt=[240, 240, 160], dt=[30, 30, 22], kr=[12, 12, 12], dr=[2.0, 2.0, 2.0], null_damping=0.15) {
  MoveL(Pose(xyz=[0.275, 0.045, 0.125], rpy=[3.14159, 0.0, 0.0]), vel=0.30, acc=0.30)
}

Gripper(action=CLOSE, wait=true)
Wait(duration=0.30)
MoveL(Pose(xyz=[0.275, 0.000, 0.175], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45)
MoveJ(READY, vel=0.50, acc=0.50)
""",
    "2. Blended MoveL + MoveC Continuous Loop": """# Demonstrates G2/C3 blended transitions without stopping at intermediate corners.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

MoveJ(READY, vel=0.55, acc=0.55)
MoveL(Pose(xyz=[0.310, -0.030, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.50, acc=0.50, blend=0.015)
MoveL(Pose(xyz=[0.310,  0.035, 0.165], rpy=[3.14159, 0.0, 0.0]), vel=0.50, acc=0.50, blend=0.015)
MoveC(center=[0.275, 0.035, 0.165], axis=[0, 0, 1], angle=1.5708, vel=0.40, acc=0.40, blend=0.012)
MoveL(Pose(xyz=[0.2757, 0.000, 0.1594], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45)
""",
    "3. Discrete Waypoint Interpolation (MovePVA)": """# Demonstrates C3 Septic Hermite (P, V, A) and Cartesian-twist (P, V) interpolation.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]
MoveJ(READY, vel=0.50, acc=0.50)

MovePVA(space=JOINT, vel=0.50, acc=0.50) {
  point(p=[0.25, 0.40, 0.95, 0.0, 0.55, 0.0], v=[0.30, 0.10, -0.10, 0.0, -0.10, 0.0], a=[0.4, 0.0, 0.0, 0.0, 0.0, 0.0])
  point(p=[0.45, 0.45, 0.92, 0.0, 0.47, 0.0], v=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  point(p=READY)
}

MovePVA(space=CARTESIAN, vel=0.40, acc=0.40) {
  point(p=Pose(xyz=[0.295, 0.025, 0.175], rpy=[3.14159, 0.0, 0.0]), v=[0.0, 0.04, 0.0, 0.0, 0.0, 0.0])
  point(p=Pose(xyz=[0.2757, 0.000, 0.1594], rpy=[3.14159, 0.0, 0.0]))
}
""",
    "4. Cartesian Impedance Surface Wipe": """# Demonstrates smooth transition into compliant Cartesian impedance tracking and back.
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]
MoveJ(READY, vel=0.50, acc=0.50)

ImpedanceCartesian(kt=[200, 200, 120], dt=[28, 28, 20], kr=[10, 10, 10], dr=[1.8, 1.8, 1.8], null_damping=0.15) {
  MoveL(Pose(xyz=[0.295, -0.040, 0.155], rpy=[3.14159, 0.0, 0.0]), vel=0.35, acc=0.35, blend=0.012)
  MoveL(Pose(xyz=[0.295,  0.040, 0.155], rpy=[3.14159, 0.0, 0.0]), vel=0.35, acc=0.35)
}

Wait(duration=0.25)
MoveJ(READY, vel=0.45, acc=0.45)
""",
    "5. Compliant Stack & Side-Grasp Arc (Demo)": """# Full physical manipulation demo matching scene.xml object positions:
# - Cylinder at [0.24, -0.15, 0.05] (height 100 mm, dia 30 mm)
# - Box at [0.24, 0.12, 0.04] (30 x 40 x 80 mm)
joint ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Exit wrist singularity (joint5=0 at ZERO) before running Cartesian MoveL
joint READY = [0.0, 0.35, 1.0, 0.0, 0.65, 0.0]

MoveJ(READY, vel=0.55, acc=0.55)
Gripper(action=OPEN, wait=true)

# 1. MoveL directly above cylinder and stop
MoveL(Pose(xyz=[0.240, -0.150, 0.160], rpy=[3.14159, 0.0, 0.0]), vel=0.45, acc=0.45)

# 2. Descend to z=85 mm and close gripper to pick cylinder
MoveL(Pose(xyz=[0.240, -0.150, 0.085], rpy=[3.14159, 0.0, 0.0]), vel=0.28, acc=0.30)
Gripper(action=CLOSE, wait=true)
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
ImpedanceCartesian(kt=[450, 450, 100], dt=[40, 40, 20], kr=[25, 25, 18], dr=[3.0, 3.0, 2.5], null_damping=0.15) {
  MoveL(Pose(xyz=[0.240, 0.120, 0.135], rpy=[3.14159, 0.0, 0.0]), vel=0.18, acc=0.20)
}

# 5. Stop, open gripper, lift to z=200 mm, and blend MoveC to approach box from -Y side
Wait(duration=0.30)
Gripper(action=OPEN, wait=true)
Wait(duration=0.20)
MoveL(Pose(xyz=[0.240, 0.120, 0.200], rpy=[3.14159, 0.0, 0.0]), vel=0.40, acc=0.40, blend=0.025)
MoveC(
  via=Pose(xyz=[0.240, 0.055, 0.140], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, 0.045, 0.055], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.35, acc=0.35
)

# Approach from the side without palm collision and close gripper across the 30 mm X-width
MoveL(Pose(xyz=[0.240, 0.112, 0.055], rpy=[3.14159, 1.309, -1.5708]), vel=0.22, acc=0.25)
Gripper(action=CLOSE, wait=true)
Wait(duration=0.25)

# 6. Large vertical MoveC arc (rise up, then descend to ~3.5 cm above tabletop)
MoveC(
  via=Pose(xyz=[0.250, 0.020, 0.215], rpy=[3.14159, 1.309, -1.5708]),
  target=Pose(xyz=[0.240, -0.070, 0.090], rpy=[3.14159, 1.309, -1.5708]),
  vel=0.40, acc=0.40
)

# 7. Compliant Cartesian impedance placement onto table, open gripper, retract clear, MoveJ ZERO
ImpedanceCartesian(kt=[450, 450, 120], dt=[40, 40, 22], kr=[25, 25, 18], dr=[3.0, 3.0, 2.5], null_damping=0.15) {
  MoveL(Pose(xyz=[0.240, -0.070, 0.038], rpy=[3.14159, 1.309, -1.5708]), vel=0.18, acc=0.20)
}
Gripper(action=OPEN, wait=true)
Wait(duration=0.20)
MoveL(Pose(xyz=[0.235, -0.118, 0.090], rpy=[3.14159, 1.309, -1.5708]), vel=0.35, acc=0.35)
MoveL(Pose(xyz=[0.200, -0.150, 0.240], rpy=[3.14159, 0.0, 0.0]), vel=0.35, acc=0.35)
MoveJ(ZERO, vel=0.50, acc=0.50)
""",
}
