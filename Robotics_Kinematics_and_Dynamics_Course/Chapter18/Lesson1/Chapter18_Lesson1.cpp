#include <cmath>
#include <vector>

struct Vec2 {
    double x;
    double y;
};

struct Planar2R {
    double l1;
    double l2;

    Vec2 fk(double q1, double q2) const {
        Vec2 p;
        double q12 = q1 + q2;
        p.x = l1 * std::cos(q1) + l2 * std::cos(q12);
        p.y = l1 * std::sin(q1) + l2 * std::sin(q12);
        return p;
    }
};

int main() {
    Planar2R robot{1.0, 0.8};

    double T = 2.0;
    std::size_t N = 200;
    double dt = T / static_cast<double>(N);

    // Parameters for joint polynomials
    double q10 = 0.0, q20 = 0.5;
    double a1 = 0.8, a2 = -0.3;
    double b1 = -0.2, b2 = 0.15;

    std::vector<double> t(N + 1);
    std::vector<Vec2> xtraj(N + 1);

    for (std::size_t k = 0; k <= N; ++k) {
        double tk = k * dt;
        t[k] = tk;

        double q1 = q10 + a1 * tk + b1 * tk * tk;
        double q2 = q20 + a2 * tk + b2 * tk * tk;

        xtraj[k] = robot.fk(q1, q2);
    }

    // xtraj[k].x, xtraj[k].y now hold the task-space samples
    // (Export to file or use in further computations)

    return 0;
}
      
