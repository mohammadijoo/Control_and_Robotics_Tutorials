"""
Chapter5_Lesson1_Ex1.py
Exercise: numerical wheel odometry computation from a short tick sequence.

Run:
  python Chapter5_Lesson1_Ex1.py
"""

from Chapter5_Lesson1 import DiffDriveParams, DifferentialDriveOdometry, Pose2D

# Parameters (example)
params = DiffDriveParams(r_l=0.06, r_r=0.06, b=0.34, ticks_per_rev=4096, gear_ratio=1.0)
odo = DifferentialDriveOdometry(params, pose0=Pose2D(0.0, 0.0, 0.0))

# (dN_L, dN_R) tick increments over 5 samples
seq = [(120, 120), (120, 150), (120, 150), (120, 150), (120, 120)]

for k, (dL, dR) in enumerate(seq, start=1):
    pose = odo.update_from_ticks(dL, dR)
    print(f"k={k}  x={pose.x:.4f}  y={pose.y:.4f}  theta={pose.theta:.4f} rad")
