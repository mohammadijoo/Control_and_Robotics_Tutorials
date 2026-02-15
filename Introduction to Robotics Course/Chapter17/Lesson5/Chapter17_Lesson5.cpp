#include <iostream>
#include <Eigen/Dense>

using State = Eigen::Vector2d; // [e_y, e_psi]

struct RowFollower {
    double T;   // sampling period
    double v;   // forward speed
    Eigen::RowVector2d K; // [k1, k2]

    RowFollower(double T_, double v_, double k1, double k2)
        : T(T_), v(v_), K(k1, k2) {}

    // One-step update of state and control
    double computeControl(const State& x) const {
        // u = -K x
        return -K * x;
    }

    State propagate(const State& x, double u) const {
        // x_{k+1} = A x_k + B u_k
        Eigen::Matrix2d A;
        A << 1.0, T * v,
              0.0, 1.0;
        Eigen::Vector2d B;
        B << 0.0, T;

        return A * x + B * u;
    }
};

int main() {
    double T = 0.1;   // 0.1 s sample time
    double v = 1.0;   // 1 m/s forward speed
    double k1 = 0.5;  // lateral gain
    double k2 = 1.0;  // heading gain

    RowFollower controller(T, v, k1, k2);

    State x;
    x << 0.5, 0.2; // initial errors: 0.5 m lateral, 0.2 rad heading

    for (int k = 0; k < 50; ++k) {
        double u = controller.computeControl(x);
        x = controller.propagate(x, u);
        std::cout << "k=" << k
                  << " e_y=" << x(0)
                  << " e_psi=" << x(1)
                  << " u=" << u << std::endl;
    }
    return 0;
}
      
