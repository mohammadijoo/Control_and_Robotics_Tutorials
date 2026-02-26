import networkx as nx

G = nx.DiGraph()

# Add nodes and weighted edges
for v, nbrs in graph.items():
    G.add_node(v)
    for w, cost in nbrs:
        G.add_edge(v, w, weight=cost)

# Shortest path with Dijkstra
path = nx.shortest_path(G, source=start, target=goal, weight="weight")
cost = nx.shortest_path_length(G, source=start, target=goal, weight="weight")
print("networkx path:", path)
print("networkx cost:", cost)
      
