import numpy as np
import networkx as nx

# -----------------------------
# 1. Candidate projects and metrics
# -----------------------------
projects = [
    {
        "name": "Kinodynamic RRT* with risk-sensitive cost",
        "I": 0.8,   # impact (0..1 heuristic)
        "A": 0.9,   # alignment
        "D": 0.7,   # difficulty
        "R": 0.6    # risk
    },
    {
        "name": "Task and motion planning for pick-place",
        "I": 0.7,
        "A": 0.8,
        "D": 0.5,
        "R": 0.4
    },
    {
        "name": "Multi-robot swarm coverage with formal specs",
        "I": 0.9,
        "A": 0.7,
        "D": 0.9,
        "R": 0.8
    }
]

def min_max_normalize(values, eps=1e-8):
    v = np.asarray(values, dtype=float)
    vmin, vmax = v.min(), v.max()
    return (v - vmin) / (vmax - vmin + eps)

# Extract raw metric vectors
I_raw = [p["I"] for p in projects]
A_raw = [p["A"] for p in projects]
D_raw = [p["D"] for p in projects]
R_raw = [p["R"] for p in projects]

# Normalize to [0,1]
I_norm = min_max_normalize(I_raw)
A_norm = min_max_normalize(A_raw)
D_norm = min_max_normalize(D_raw)
R_norm = min_max_normalize(R_raw)

for idx, p in enumerate(projects):
    p["I_norm"] = float(I_norm[idx])
    p["A_norm"] = float(A_norm[idx])
    p["D_norm"] = float(D_norm[idx])
    p["R_norm"] = float(R_norm[idx])

# Weight vector lambda
lam = {
    "I": 0.4,   # importance of impact
    "A": 0.3,   # importance of alignment
    "D": 0.2,   # penalty on difficulty
    "R": 0.1    # penalty on risk
}

def utility(p, lam):
    return (
        lam["I"] * p["I_norm"]
        + lam["A"] * p["A_norm"]
        - lam["D"] * p["D_norm"]
        - lam["R"] * p["R_norm"]
    )

best = None
best_score = -1e9
for p in projects:
    score = utility(p, lam)
    p["U"] = score
    if score > best_score:
        best_score = score
        best = p

print("Best project under lam =", lam, "is:")
print("\t", best["name"], "with score", best["U"])

# -----------------------------
# 2. Citation graph analysis
# -----------------------------
G = nx.DiGraph()

# Nodes: toy IDs mapped to canonical papers
G.add_nodes_from([
    (1, {"title": "RRT* optimal kinodynamic planning"}),
    (2, {"title": "PRM* and asymptotic optimality"}),
    (3, {"title": "Risk-sensitive motion planning"}),
    (4, {"title": "Formal methods for motion planning"}),
    (5, {"title": "RL for continuous robot control"})
])

# Add directed citation edges (i cites j)
edges = [
    (1, 2),
    (3, 1),
    (3, 2),
    (4, 1),
    (4, 3),
    (5, 3),
    (5, 2)
]
G.add_edges_from(edges)

# PageRank centrality
pagerank = nx.pagerank(G, alpha=0.9)
print("\nPageRank centrality:")
for node_id, score in sorted(pagerank.items()):
    print(node_id, G.nodes[node_id]["title"], ":", round(score, 3))

# r-neighborhood coverage: given a set of seed papers
def r_neighborhood(G, seeds, r):
    """Return set of nodes within graph distance <= r from seeds."""
    out = set()
    for s in seeds:
        out.add(s)
        # BFS up to depth r
        frontier = {s}
        for _ in range(r):
            new_frontier = set()
            for u in frontier:
                for v in G.successors(u):
                    if v not in out:
                        out.add(v)
                        new_frontier.add(v)
            frontier = new_frontier
    return out

def coverage(G, seeds, r):
    n = G.number_of_nodes()
    return len(r_neighborhood(G, seeds, r)) / float(n)

# Suppose your project builds mainly on papers 1 and 3
seeds = [1, 3]
cov_r1 = coverage(G, seeds, r=1)
cov_r2 = coverage(G, seeds, r=2)
print("\nCoverage with seeds", seeds)
print("r=1:", cov_r1)
print("r=2:", cov_r2)
      
