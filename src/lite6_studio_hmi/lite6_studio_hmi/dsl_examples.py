"""Built-in Lite6Script example programs for Lite6 Studio HMI."""

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
}
