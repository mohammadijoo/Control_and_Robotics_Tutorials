// Chapter18_Lesson5.cpp
// Case Study: Agricultural / Delivery Robots
// Composite-cost Dijkstra planner for outdoor AMR case studies

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

struct Edge {
    int u, v;
    double length_m;
    double slope_rad;
    double roughness;
    double slip_risk;
    double speed_limit;
};

double edgeTime(const Edge& e, double v_cmd) {
    double v = std::max(0.2, std::min(v_cmd, e.speed_limit));
    return e.length_m / v;
}

double edgeEnergy(const Edge& e, double mass, double v_cmd) {
    const double g = 9.81;
    double v = std::max(0.2, std::min(v_cmd, e.speed_limit));
    double c_rr = 0.03;
    double rho_air = 1.2;
    double CdA = 0.45;
    double P_aux = 80.0;

    double F_roll = c_rr * mass * g * std::cos(e.slope_rad);
    double F_grade = mass * g * std::sin(e.slope_rad);
    double F_drag = 0.5 * rho_air * CdA * v * v;
    double P = std::max(0.0, (F_roll + F_grade + F_drag) * v) + P_aux;
    return P * (e.length_m / v); // Joules
}

double edgeRisk(const Edge& e) {
    return 0.45 * e.roughness + 0.55 * e.slip_risk + 0.15 * std::abs(e.slope_rad);
}

double safeSpeedFromClearance(double clearance_m, double reaction_s, double mu, double margin = 0.5) {
    double d = std::max(0.0, clearance_m - margin);
    double a = 1.0 / (2.0 * mu * 9.81);
    double b = reaction_s;
    double c = -d;
    double disc = std::max(0.0, b*b - 4.0*a*c);
    return (-b + std::sqrt(disc)) / (2.0*a);
}

struct NodeState {
    double cost;
    int node;
    bool operator>(const NodeState& other) const { return cost > other.cost; }
};

std::pair<std::vector<int>, double> dijkstraComposite(
    const std::vector<Edge>& edges,
    int n_nodes,
    int source,
    int target,
    double mass,
    double v_cmd,
    double wt, double we, double wr,
    double reaction_s, double mu, double clearance_default
) {
    std::vector<std::vector<Edge>> g(n_nodes);
    for (const auto& e : edges) {
        g[e.u].push_back(e);
    }

    std::vector<double> dist(n_nodes, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n_nodes, -1);
    dist[source] = 0.0;

    std::priority_queue<NodeState, std::vector<NodeState>, std::greater<NodeState>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        NodeState cur = pq.top();
        pq.pop();
        if (cur.cost > dist[cur.node]) continue;
        if (cur.node == target) break;

        for (const auto& e : g[cur.node]) {
            double v_safe = safeSpeedFromClearance(clearance_default, reaction_s, mu);
            double v_use = std::min(v_cmd, std::min(e.speed_limit, std::max(0.25, v_safe)));

            double c = wt * edgeTime(e, v_use)
                     + we * (edgeEnergy(e, mass, v_use) / 1000.0)
                     + wr * edgeRisk(e);

            double nd = dist[cur.node] + c;
            if (nd < dist[e.v]) {
                dist[e.v] = nd;
                parent[e.v] = cur.node;
                pq.push({nd, e.v});
            }
        }
    }

    std::vector<int> path;
    if (!std::isfinite(dist[target])) return {path, dist[target]};
    for (int v = target; v != -1; v = parent[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return {path, dist[target]};
}

int main() {
    std::vector<Edge> edges = {
        {0,1,25, 0.02,0.10,0.15,2.5},
        {1,2,18, 0.05,0.30,0.35,1.8},
        {0,3,20,-0.01,0.08,0.10,2.0},
        {3,2,24, 0.03,0.12,0.20,2.2},
        {2,4,30, 0.01,0.15,0.18,3.0},
        {3,4,28, 0.07,0.40,0.45,1.5}
    };

    auto ag = dijkstraComposite(edges, 5, 0, 4, 180.0, 1.7, 1.0, 0.03, 8.0, 0.4, 0.55, 2.5);
    std::cout << "Agricultural path: ";
    for (int n : ag.first) std::cout << n << " ";
    std::cout << " | cost = " << ag.second << "\n";

    auto del = dijkstraComposite(edges, 5, 0, 4, 45.0, 2.2, 1.5, 0.01, 5.0, 0.6, 0.65, 3.5);
    std::cout << "Delivery path: ";
    for (int n : del.first) std::cout << n << " ";
    std::cout << " | cost = " << del.second << "\n";

    double vmax = safeSpeedFromClearance(4.0, 0.5, 0.6, 0.6);
    std::cout << "Safe speed from clearance = " << vmax << " m/s\n";

    return 0;
}
