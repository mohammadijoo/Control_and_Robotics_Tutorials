#include <vector>
#include <random>
#include <limits>
#include <Eigen/Dense>

using Config = Eigen::VectorXd;

struct Node {
    Config q;
    int parent; // -1 for root
};

struct Tree {
    std::vector<Node> nodes;

    int addNode(const Config& q, int parent) {
        nodes.push_back(Node{q, parent});
        return static_cast<int>(nodes.size()) - 1;
    }

    int nearest(const Config& q) const {
        double best = std::numeric_limits<double>::infinity();
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            double d = (nodes[i].q - q).norm();
            if (d < best) {
                best = d;
                best_idx = i;
            }
        }
        return best_idx;
    }
};

Config steer(const Config& q_near, const Config& q_rand, double eta) {
    Config d = q_rand - q_near;
    double dist = d.norm();
    if (dist <= eta) {
        return q_rand;
    }
    return q_near + (eta / dist) * d;
}

// Collision checker signature; use FCL, bullet, etc. in practice.
bool collisionFree(const Config& q1, const Config& q2);

enum class ExtendStatus { Trapped, Advanced, Reached };

ExtendStatus extend(Tree& T, const Config& q_rand, double eta, int& new_idx) {
    int idx_near = T.nearest(q_rand);
    Config q_near = T.nodes[idx_near].q;
    Config q_new = steer(q_near, q_rand, eta);
    if (!collisionFree(q_near, q_new)) {
        return ExtendStatus::Trapped;
    }
    new_idx = T.addNode(q_new, idx_near);
    if ((q_new - q_rand).norm() < 1e-6) {
        return ExtendStatus::Reached;
    }
    return ExtendStatus::Advanced;
}
      
