import numpy as np

# -----------------------------
# Robot and workspace geometry
# -----------------------------
L1, L2 = 1.0, 0.7  # link lengths

# Axis-aligned rectangular obstacles in workspace:
# each obstacle is (xmin, xmax, ymin, ymax)
obstacles = [
    (-0.3, 0.3, 0.5, 0.8),
    (0.6, 1.0, -0.2, 0.1),
]

def fk_2r(theta1, theta2):
    """
    Forward kinematics for a planar 2R arm.
    Returns positions of joints and end-effector in workspace.
    """
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return np.array([[0.0, 0.0],
                     [x1, y1],
                     [x2, y2]])

def seg_intersects_aabb(p, q, aabb):
    """
    Check whether segment pq intersects an axis-aligned bounding box (AABB).
    Using Liang-Barsky style clipping for robustness.
    """
    xmin, xmax, ymin, ymax = aabb

    dx = q[0] - p[0]
    dy = q[1] - p[1]

    t0, t1 = 0.0, 1.0

    def clip(p, q, r0, r1):
        if q == 0.0:
            return (True, r0, r1) if p <= r1 and p >= r0 else (False, r0, r1)
        t_min = (r0 - p) / q
        t_max = (r1 - p) / q
        if t_min > t_max:
            t_min, t_max = t_max, t_min
        return True, t_min, t_max

    ok, t_min_x, t_max_x = clip(p[0], dx, xmin, xmax)
    if not ok:
        return False
    t0 = max(t0, t_min_x)
    t1 = min(t1, t_max_x)
    if t0 > t1:
        return False

    ok, t_min_y, t_max_y = clip(p[1], dy, ymin, ymax)
    if not ok:
        return False
    t0 = max(t0, t_min_y)
    t1 = min(t1, t_max_y)
    if t0 > t1:
        return False

    return True

def is_collision(theta1, theta2):
    pts = fk_2r(theta1, theta2)
    segs = [(pts[0], pts[1]), (pts[1], pts[2])]
    for obs in obstacles:
        for p, q in segs:
            if seg_intersects_aabb(p, q, obs):
                return True
    return False

# -----------------------------
# C-space grid
# -----------------------------
N = 200  # resolution per joint
theta1_vals = np.linspace(-np.pi, np.pi, N)
theta2_vals = np.linspace(-np.pi, np.pi, N)

# occupancy: 0 = free, 1 = colliding
grid = np.zeros((N, N), dtype=np.uint8)

for i, t1 in enumerate(theta1_vals):
    for j, t2 in enumerate(theta2_vals):
        grid[i, j] = 1 if is_collision(t1, t2) else 0

# Optional: visualize with matplotlib (not shown here in HTML)
# import matplotlib.pyplot as plt
# plt.imshow(grid.T, origin="lower", extent=[-np.pi, np.pi, -np.pi, np.pi])
# plt.xlabel("theta1"), plt.ylabel("theta2")
# plt.show()

# Note:
# In a larger system, you might integrate this grid with OMPL (Python bindings)
# by creating a StateValidityChecker that queries 'grid' instead of calling
# full geometric collision checks every time.
      
