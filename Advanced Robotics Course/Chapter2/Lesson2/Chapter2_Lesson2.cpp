#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <functional>

struct Edge {
    int to;
    double cost;
};

using Graph = std::vector<std::vector<Edge>>;

struct NodeRecord {
    int v;
    double f;
    double g;
    bool operator<(const NodeRecord& other) const {
        // reversed for min-heap behavior
        return f > other.f;
    }
};

std::pair<double, std::vector<int>>
astar(const Graph& graph,
      int start,
      int goal,
      const std::function<double(int)>& heuristic)
{
    const double INF = std::numeric_limits<double>::infinity();
    std::priority_queue<NodeRecord> open;
    std::vector<double> g(graph.size(), INF);
    std::vector<int> parent(graph.size(), -1);
    std::vector<bool> closed(graph.size(), false);

    g[start] = 0.0;
    open.push({start, heuristic(start), 0.0});

    while (!open.empty()) {
        NodeRecord rec = open.top();
        open.pop();
        int v = rec.v;

        if (closed[v]) continue;
        if (v == goal) {
            // reconstruct path
            std::vector<int> path;
            for (int u = goal; u != -1; u = parent[u])
                path.push_back(u);
            std::reverse(path.begin(), path.end());
            return {rec.g, path};
        }

        closed[v] = true;
        for (const Edge& e : graph[v]) {
            if (e.cost < 0.0) {
                throw std::runtime_error("Negative edge cost not allowed.");
            }
            double g_tent = rec.g + e.cost;
            if (g_tent < g[e.to]) {
                g[e.to] = g_tent;
                parent[e.to] = v;
                double f = g_tent + heuristic(e.to);
                open.push({e.to, f, g_tent});
            }
        }
    }
    throw std::runtime_error("No path found.");
}
      
