#include <vector>
#include <queue>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp> // robotics kinematics library

struct Edge {
    int to;
    double cost;
};

using Graph = std::vector< std::vector<Edge> >;

std::vector<int> dijkstra(const Graph& G, int start, int goal) {
    const double INF = std::numeric_limits<double>::infinity();
    int n = static_cast<int>(G.size());
    std::vector<double> dist(n, INF);
    std::vector<int> parent(n, -1);
    using Node = std::pair<double,int>;
    auto cmp = [](const Node& a, const Node& b) { return a.first > b.first; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);

    dist[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [d,u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        if (u == goal) break;
        for (const auto& e : G[u]) {
            double nd = d + e.cost;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = u;
                pq.push({nd, e.to});
            }
        }
    }

    std::vector<int> path;
    if (dist[goal] == INF) return path;
    for (int v = goal; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    // Example: 4 grasps with a few feasible transitions
    Graph G(4);
    G[0].push_back({1, 1.0});
    G[1].push_back({2, 1.0});
    G[0].push_back({2, 2.5});
    G[2].push_back({3, 1.0});

    auto path = dijkstra(G, 0, 3);
    if (path.empty()) {
        std::cout << "No regrasp path.\n";
    } else {
        std::cout << "Path:";
        for (int v : path) {
            std::cout << " " << v;
        }
        std::cout << std::endl;
    }
    return 0;
}
      
