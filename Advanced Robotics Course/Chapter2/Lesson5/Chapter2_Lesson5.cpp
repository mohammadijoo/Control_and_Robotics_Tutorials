#include <array>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

constexpr int DOF = 7;

struct Node {
    std::array<int, DOF> k;
};

struct NodeHash {
    std::size_t operator()(Node const& n) const noexcept {
        std::size_t h = 0;
        for (int i = 0; i < DOF; ++i) {
            h = h * 131u + static_cast<std::size_t>(n.k[i]);
        }
        return h;
    }
};

struct NodeEq {
    bool operator()(Node const& a, Node const& b) const noexcept {
        for (int i = 0; i < DOF; ++i)
            if (a.k[i] != b.k[i]) return false;
        return true;
    }
};

struct JointLimit {
    double q_min;
    double q_max;
    double delta;
};

static JointLimit JOINT_LIMITS[DOF] = {
    {-2.9, 2.9, 0.1},
    {-2.0, 2.0, 0.1},
    {-2.0, 2.0, 0.1},
    {-2.9, 2.9, 0.1},
    {-2.0, 2.0, 0.1},
    {-2.0, 2.0, 0.1},
    {-3.1, 3.1, 0.1}
};

std::array<double, DOF> undisc(Node const& n) {
    std::array<double, DOF> q{};
    for (int i = 0; i < DOF; ++i) {
        q[i] = JOINT_LIMITS[i].q_min + n.k[i] * JOINT_LIMITS[i].delta;
    }
    return q;
}

bool isCollisionFree(std::array<double, DOF> const& q) {
    // TODO: call MoveIt/OMPL collision checker here.
    return true;
}

std::vector<Node> neighbors(Node const& n) {
    std::vector<Node> nbrs;
    for (int i = 0; i < DOF; ++i) {
        for (int step : {-1, 1}) {
            Node nb = n;
            nb.k[i] += step;
            int max_k = static_cast<int>(
                std::round((JOINT_LIMITS[i].q_max - JOINT_LIMITS[i].q_min)
                           / JOINT_LIMITS[i].delta)
            );
            if (nb.k[i] < 0 || nb.k[i] > max_k) continue;
            auto q = undisc(nb);
            if (isCollisionFree(q)) {
                nbrs.push_back(nb);
            }
        }
    }
    return nbrs;
}

double edgeCost(Node const& a, Node const& b) {
    auto qa = undisc(a);
    auto qb = undisc(b);
    double cost = 0.0;
    for (int i = 0; i < DOF; ++i)
        cost += std::abs(qb[i] - qa[i]);
    return cost;
}

double h(Node const& n, Node const& goal) {
    auto q = undisc(n);
    auto qg = undisc(goal);
    double s = 0.0;
    for (int i = 0; i < DOF; ++i) {
        double d = q[i] - qg[i];
        s += d * d;
    }
    return std::sqrt(s);
}

struct PQItem {
    double f;
    Node node;
    bool operator<(PQItem const& other) const {
        // reverse for min-heap in std::priority_queue
        return f > other.f;
    }
};

std::vector<Node> astar(Node const& start, Node const& goal) {
    std::priority_queue<PQItem> open;
    open.push({0.0, start});

    std::unordered_map<Node, double, NodeHash, NodeEq> g_score;
    std::unordered_map<Node, Node, NodeHash, NodeEq> parent;
    g_score[start] = 0.0;

    while (!open.empty()) {
        PQItem top = open.top();
        open.pop();
        Node current = top.node;

        if (NodeEq{}(current, goal)) {
            std::vector<Node> path;
            path.push_back(current);
            auto it = parent.find(current);
            while (it != parent.end()) {
                current = it->second;
                path.push_back(current);
                it = parent.find(current);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto& nb : neighbors(current)) {
            double tentative_g = g_score[current] + edgeCost(current, nb);
            auto it = g_score.find(nb);
            if (it == g_score.end() || tentative_g < it->second) {
                g_score[nb] = tentative_g;
                parent[nb] = current;
                double f = tentative_g + h(nb, goal);
                open.push({f, nb});
            }
        }
    }
    throw std::runtime_error("No path found");
}
      
