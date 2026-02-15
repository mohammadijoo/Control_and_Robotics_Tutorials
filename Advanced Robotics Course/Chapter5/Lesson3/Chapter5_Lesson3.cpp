#include <vector>
#include <cmath>
#include <limits>
#include <random>

struct State {
    double x, y, vx, vy;
};

struct Node {
    State state;
    int parent;    // index of parent in vector<Node>
    double cost;
};

double distance(const State& a, const State& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dvx = a.vx - b.vx;
    double dvy = a.vy - b.vy;
    double w_pos = 1.0;
    double w_vel = 0.1;
    return std::sqrt(w_pos * (dx * dx + dy * dy) +
                     w_vel * (dvx * dvx + dvy * dvy));
}

State propagate(const State& s, double ux, double uy,
                double dt, int steps) {
    State x = s;
    for (int k = 0; k < steps; ++k) {
        x.x  += dt * x.vx;
        x.y  += dt * x.vy;
        x.vx += dt * ux;
        x.vy += dt * uy;
    }
    return x;
}

// Collision and neighbor search code omitted for brevity.

int main() {
    std::mt19937 gen(123);
    std::uniform_real_distribution<double> unif01(0.0, 1.0);

    // Bounds
    double xmin = 0.0, xmax = 10.0;
    double ymin = 0.0, ymax = 10.0;
    double vmax = 2.0;

    std::vector<Node> tree;
    Node root;
    root.state = {1.0, 1.0, 0.0, 0.0};
    root.parent = -1;
    root.cost = 0.0;
    tree.push_back(root);

    std::vector<std::pair<double, double>> u_set = {
        {-1.0, 0.0}, {1.0, 0.0},
        {0.0, -1.0}, {0.0, 1.0},
        {0.0, 0.0}
    };

    const int maxIter = 1000;
    const double dt = 0.1;
    const int steps = 5;
    const int dim = 4;
    const double gamma = 30.0;

    for (int n = 1; n <= maxIter; ++n) {
        State x_rand;
        x_rand.x  = xmin + (xmax - xmin) * unif01(gen);
        x_rand.y  = ymin + (ymax - ymin) * unif01(gen);
        x_rand.vx = -vmax + 2.0 * vmax * unif01(gen);
        x_rand.vy = -vmax + 2.0 * vmax * unif01(gen);

        // Find nearest
        int idx_near = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (int i = 0; i < (int)tree.size(); ++i) {
            double d = distance(tree[i].state, x_rand);
            if (d < best_d) {
                best_d = d;
                idx_near = i;
            }
        }

        // Pick best control (naive)
        const State& x_near = tree[idx_near].state;
        State x_new;
        double best_edge_cost = std::numeric_limits<double>::infinity();
        for (const auto& u : u_set) {
            State cand = propagate(x_near, u.first, u.second, dt, steps);
            // Check bounds and collisions here ...
            double edge_cost = dt * steps;
            if (edge_cost < best_edge_cost) {
                best_edge_cost = edge_cost;
                x_new = cand;
            }
        }

        if (!std::isfinite(best_edge_cost)) {
            continue;
        }

        // Compute radius r_n
        double rn = std::pow(gamma * std::log((double)n) / (double)n, 1.0 / dim);

        // TODO: find neighbors within rn, choose best parent, and rewire.

        Node newNode;
        newNode.state = x_new;
        newNode.parent = idx_near;
        newNode.cost = tree[idx_near].cost + best_edge_cost;
        tree.push_back(newNode);
    }

    return 0;
}
      
