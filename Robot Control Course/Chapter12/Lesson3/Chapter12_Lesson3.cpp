
#include <iostream>
#include <Eigen/Dense>

int main() {
    const double T_fast = 0.001;   // 1 kHz
    const int m = 10;              // outer loop at 100 Hz
    const double T_slow = m * T_fast;
    const double tf = 1.0;
    const int N_steps = static_cast<int>(tf / T_fast);

    // Double-integrator discrete model
    Eigen::Matrix2d A_f;
    A_f << 1.0, T_fast,
           0.0, 1.0;
    Eigen::Vector2d B_f;
    B_f << 0.5 * T_fast * T_fast,
           T_fast;

    // Gains
    Eigen::RowVector2d K_fast;
    K_fast << 0.0, 5.0;
    double K_slow = 50.0;

    Eigen::Vector2d x;  // [q, qdot]
    x.setZero();
    double v_slow = 0.0;

    auto q_ref = [](double t) {
        return (t > 0.1) ? 0.5 : 0.0;
    };

    for (int k = 0; k < N_steps; ++k) {
        double t = k * T_fast;

        if (k % m == 0) {
            double q_des = q_ref(t);
            double e_q = q_des - x(0);
            v_slow = K_slow * e_q;
        }

        double u = v_slow - (K_fast * x)(0);
        x = A_f * x + B_f * u;
    }

    std::cout << "Final joint position: " << x(0) << std::endl;
    return 0;
}
