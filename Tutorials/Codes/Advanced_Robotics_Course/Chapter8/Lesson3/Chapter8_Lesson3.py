import numpy as np
import networkx as nx
import pinocchio as pin  # robotics kinematics library

class Grasp:
    def __init__(self, idx, T_ho, quality):
        self.idx = idx
        self.T_ho = T_ho  # 4x4 homogeneous transform (hand frame to object)
        self.quality = quality

def random_transform(max_pos=0.05):
    """Small random transform around the palm."""
    T = np.eye(4)
    T[:3, 3] = max_pos * (np.random.rand(3) - 0.5)
    # small random rotation via axis-angle
    axis = np.random.rand(3) - 0.5
    axis /= np.linalg.norm(axis) + 1e-9
    theta = 0.3 * (np.random.rand() - 0.5)
    R = pin.exp3(axis * theta)
    T[:3, :3] = R
    return T

def pose_distance(T1, T2, w_rot=1.0, w_pos=1.0):
    dp = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    R_rel = T1[:3, :3].T @ T2[:3, :3]
    tr = np.trace(R_rel)
    tr = np.clip(tr, -1.0, 3.0)
    dtheta = np.arccos((tr - 1.0) / 2.0)
    return w_pos * dp + w_rot * dtheta

# Build synthetic grasp set
np.random.seed(0)
N = 6
grasps = []
for i in range(N):
    T = random_transform()
    quality = np.random.rand()
    grasps.append(Grasp(i, T, quality))

# Build directed regrasp graph
G = nx.DiGraph()
for g in grasps:
    G.add_node(g.idx)

max_step = 0.08  # admissible pose distance for direct regrasp
for i in range(N):
    for j in range(N):
        if i == j:
            continue
        d = pose_distance(grasps[i].T_ho, grasps[j].T_ho)
        if d <= max_step:
            # edge cost: pose distance plus penalty for quality loss
            cost = d + max(0.0, grasps[i].quality - grasps[j].quality)
            G.add_edge(i, j, cost=cost)

start_id = 0
goal_id = N - 1
if nx.has_path(G, start_id, goal_id):
    path = nx.shortest_path(G, source=start_id, target=goal_id, weight="cost")
    print("Regrasp sequence:", path)
else:
    print("No path between grasps in this random example.")
      
