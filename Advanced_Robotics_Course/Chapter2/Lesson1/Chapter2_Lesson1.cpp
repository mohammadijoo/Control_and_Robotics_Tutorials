#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>

struct Edge {
    int to;
    double cost;
};

using Graph = std::vector<std::vector<Edge> >;

std::pair<double, std::vector<int> >
dijkstra(const Graph& g, int start, int goal) {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(g.size(), INF);
    std::vector<int> parent(g.size(), -1);

    using Node = std::pair<double, int>; // (dist, vertex)
    auto cmp = [](const Node& a, const Node& b) {
        return a.first > b.first;
    };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);

    dist[start] = 0.0;
    pq.emplace(0.0, start);

    while (!pq.empty()) {
        auto [d, v] = pq.top();
        pq.pop();
        if (v == goal) break;
        if (d > dist[v]) continue;

        for (const auto& e : g[v]) {
            double nd = d + e.cost;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = v;
                pq.emplace(nd, e.to);
            }
        }
    }

    if (dist[goal] == INF) {
        return {INF, {}};
    }

    std::vector<int> path;
    for (int v = goal; v != -1; v = parent[v]) {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    return {dist[goal], path};
}

int main() {
    // Small example graph
    Graph g(4);
    g[0].push_back({1, 1.0});
    g[0].push_back({2, 2.0});
    g[1].push_back({3, 1.0});
    g[2].push_back({3, 0.5});

    auto [cost, path] = dijkstra(g, 0, 3);
    std::cout << "Cost: " << cost << "\nPath:";
    for (int v : path) std::cout << " " << v;
    std::cout << std::endl;
    return 0;
}
      
