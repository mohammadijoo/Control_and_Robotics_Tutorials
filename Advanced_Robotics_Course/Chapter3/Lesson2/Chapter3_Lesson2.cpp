#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <queue>
#include <utility>

struct Point2D {
    double x;
    double y;
};

struct Box {
    double xmin, ymin, xmax, ymax;
};

using Edge = std::pair<int, double>;
using Graph = std::vector<std::vector<Edge> >;

bool inCollision(const Point2D &q, const std::vector<Box> &obstacles) {
    for (const auto &b : obstacles) {
        if (b.xmin <= q.x && q.x <= b.xmax &&
            b.ymin <= q.y && q.y <= b.ymax) {
            return true;
        }
    }
    return false;
}

double euclidean(const Point2D &a, const Point2D &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Point2D> interpolate(const Point2D &a, const Point2D &b, int nSteps) {
    std::vector<Point2D> path;
    path.reserve(static_cast<std::size_t>(nSteps + 1));
    for (int i = 0; i <= nSteps; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(nSteps);
        Point2D q{ (1.0 - t) * a.x + t * b.x,
                   (1.0 - t) * a.y + t * b.y };
        path.push_back(q);
    }
    return path;
}

bool edgeCollisionFree(const Point2D &a,
                       const Point2D &b,
                       const std::vector<Box> &obstacles,
                       int nSteps = 20) {
    auto path = interpolate(a, b, nSteps);
    for (const auto &q : path) {
        if (inCollision(q, obstacles)) {
            return false;
        }
    }
    return true;
}

Graph buildPRM(const std::vector<Point2D> &samples,
               double r,
               const std::vector<Box> &obstacles) {
    int n = static_cast<int>(samples.size());
    Graph graph(n);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double d = euclidean(samples[i], samples[j]);
            if (d <= r) {
                if (edgeCollisionFree(samples[i], samples[j], obstacles)) {
                    graph[i].push_back(std::make_pair(j, d));
                    graph[j].push_back(std::make_pair(i, d));
                }
            }
        }
    }
    return graph;
}

std::vector<int> dijkstra(const Graph &graph, int start, int goal) {
    int n = static_cast<int>(graph.size());
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);

    using Node = std::pair<double, int>;
    auto cmp = [](const Node &a, const Node &b) { return a.first > b.first; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);

    dist[start] = 0.0;
    pq.push(std::make_pair(0.0, start));

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (u == goal) {
            break;
        }
        if (d > dist[u]) {
            continue;
        }
        for (const auto &edge : graph[u]) {
            int v = edge.first;
            double w = edge.second;
            double nd = d + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push(std::make_pair(nd, v));
            }
        }
    }

    if (!std::isfinite(dist[goal])) {
        return {};
    }

    std::vector<int> path;
    for (int u = goal; u != -1; u = prev[u]) {
        path.push_back(u);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Lazy-PRM variant: build graph with distance-based edges only, then validate on demand.
Graph buildLazyPRM(const std::vector<Point2D> &samples, double r) {
    int n = static_cast<int>(samples.size());
    Graph graph(n);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double d = euclidean(samples[i], samples[j]);
            if (d <= r) {
                graph[i].push_back(std::make_pair(j, d));
                graph[j].push_back(std::make_pair(i, d));
            }
        }
    }
    return graph;
}

bool validatePathLazy(const std::vector<int> &path,
                      const std::vector<Point2D> &samples,
                      Graph &graph,
                      const std::vector<Box> &obstacles,
                      int nSteps = 20) {
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        int u = path[k];
        int v = path[k + 1];
        if (!edgeCollisionFree(samples[u], samples[v], obstacles, nSteps)) {
            // remove edge u-v
            auto Ν = graph[u];
            Nu.erase(std::remove_if(Nu.begin(), Nu.end(),
                                    [v](const Edge &e) { return e.first == v; }),
                     Nu.end());
            auto &Nv = graph[v];
            Nv.erase(std::remove_if(Nv.begin(), Nv.end(),
                                    [u](const Edge &e) { return e.first == u; }),
                     Nv.end());
            return false;
        }
    }
    return true;
}
      
