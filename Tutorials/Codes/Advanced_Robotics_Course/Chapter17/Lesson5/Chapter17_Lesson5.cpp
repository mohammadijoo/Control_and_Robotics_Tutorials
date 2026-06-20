#include <vector>
#include <queue>
#include <limits>
#include <cmath>

struct Vec3 {
    double x, y, z;
};

double distance(const Vec3 &a, const Vec3 &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

struct Edge {
    int to;
    double w; // edge weight
};

using Graph = std::vector<std::vector<Edge> >;

// Dijkstra shortest path from src to dst
std::vector<int> shortest_path(const Graph &G, int src, int dst) {
    const double INF = std::numeric_limits<double>::infinity();
    int n = static_cast<int>(G.size());
    std::vector<double> dist(n, INF);
    std::vector<int> parent(n, -1);

    using Node = std::pair<double,int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > pq;

    dist[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        if (u == dst) break;
        for (const Edge &e : G[u]) {
            int v = e.to;
            double nd = d + e.w;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }

    std::vector<int> path;
    if (dist[dst] == INF) return path; // no path
    for (int v = dst; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
      
