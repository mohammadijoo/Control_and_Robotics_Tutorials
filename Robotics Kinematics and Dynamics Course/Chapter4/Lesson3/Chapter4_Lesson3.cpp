#include <iostream>
#include <cmath>

struct Vec2 {
    double x;
    double y;
};

Vec2 rot2_times_vec(double theta, const Vec2 &v) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    Vec2 out;
    out.x = c * v.x - s * v.y;
    out.y = s * v.x + c * v.y;
    return out;
}

Vec2 fk_planar_2R(double theta1, double theta2,
                  double L1, double L2) {
    Vec2 e1{L1, 0.0};
    Vec2 e2{L2, 0.0};

    Vec2 p1 = rot2_times_vec(theta1, e1);
    Vec2 p2 = p1;
    Vec2 e2_rot = rot2_times_vec(theta1 + theta2, e2);
    p2.x += e2_rot.x;
    p2.y += e2_rot.y;
    return p2;
}

double fourbar_constraint(double theta1, double theta3,
                          double L0, double L1, double L2, double L3) {
    Vec2 pB{L1 * std::cos(theta1),
            L1 * std::sin(theta1)};
    Vec2 pC{L0 + L3 * std::cos(theta3),
            L3 * std::sin(theta3)};
    double dx = pB.x - pC.x;
    double dy = pB.y - pC.y;
    return dx*dx + dy*dy - L2*L2;
}

double solve_fourbar_theta3(double theta1,
                            double L0, double L1, double L2, double L3,
                            double theta3_init = 0.0,
                            double tol = 1e-10,
                            int max_iter = 50) {
    double theta3 = theta3_init;
    for (int k = 0; k < max_iter; ++k) {
        double Phi = fourbar_constraint(theta1, theta3, L0, L1, L2, L3);
        double h = 1e-6;
        double Phi_p = fourbar_constraint(theta1, theta3 + h, L0, L1, L2, L3);
        double Phi_m = fourbar_constraint(theta1, theta3 - h, L0, L1, L2, L3);
        double dPhi = (Phi_p - Phi_m) / (2.0 * h);
        if (std::fabs(dPhi) < 1e-14) {
            break;
        }
        double step = Phi / dPhi;
        theta3 -= step;
        if (std::fabs(step) < tol) {
            break;
        }
    }
    return theta3;
}

int main() {
    // Open-chain example
    double theta1 = 0.5, theta2 = -0.3;
    double L1 = 0.5, L2 = 0.4;
    Vec2 p = fk_planar_2R(theta1, theta2, L1, L2);
    std::cout << "Open-chain end-effector: ("
              << p.x << ", " << p.y << ")\n";

    // Closed-chain four-bar example
    double L0 = 0.8, L3 = 0.4, Lmid = 0.7;
    double theta1_in = 0.4;
    double theta3 = solve_fourbar_theta3(theta1_in, L0, L1, Lmid, L3, 0.2);
    std::cout << "Closed-chain theta3: " << theta3 << "\n";
    std::cout << "Constraint residual: "
              << fourbar_constraint(theta1_in, theta3, L0, L1, Lmid, L3)
              << "\n";
    return 0;
}
      
