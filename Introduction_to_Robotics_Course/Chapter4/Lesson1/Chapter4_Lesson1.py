
# Represent links and joints, compute mobility via Grübler–Kutzbach
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Joint:
    parent: int
    child: int
    f: int  # allowed DOF of joint (1 for R/P, 2 for universal, etc.)

def spatial_mobility(N: int, joints: List[Joint]) -> int:
    J = len(joints)
    return 6*(N - 1 - J) + sum(j.f for j in joints)

# Example: 6R serial arm (N=7 including base, J=6, all f_i=1)
joints = [Joint(i, i+1, 1) for i in range(6)]
print("Mobility:", spatial_mobility(7, joints))

# Optionally build a link-joint graph
import networkx as nx
G = nx.Graph()
G.add_nodes_from(range(7))
G.add_edges_from((j.parent, j.child) for j in joints)
print("Is tree?", nx.is_tree(G))
      