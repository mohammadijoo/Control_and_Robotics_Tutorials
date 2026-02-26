
import networkx as nx

# Toy modular robot: two 3-module subsystems with one bridge
G = nx.Graph()
G.add_edges_from([
    (1,2),(2,3),(1,3),   # subsystem A
    (4,5),(5,6),(4,6),   # subsystem B
    (3,4)                # bridge
])

# Partition into two communities
communities = [{1,2,3}, {4,5,6}]
Q_modular = nx.algorithms.community.quality.modularity(G, communities)

# Less modular: connect everything densely
H = nx.complete_graph([1,2,3,4,5,6])
Q_dense = nx.algorithms.community.quality.modularity(H, communities)

print("Q modular robot =", Q_modular)
print("Q dense robot   =", Q_dense)

# Cayley theorem check for small n (trees only)
def cayley_count(n):
    return n**(n-2) if n >= 2 else 1

for n in range(2,7):
    print(n, "modules: tree configs =", cayley_count(n))
      