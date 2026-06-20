#include <iostream>
#include <Eigen/Dense>

struct Planar2RModel {
    double L1, L2;
    Eigen::Vector2d q_min;
    Eigen::Vector2d q_max;

    Planar2RModel(double L1_, double L2_)
        : L1(L1_), L2(L2_) {
        q_min = Eigen::Vector2d(-M_PI, -M_PI);
        q_max = Eigen::Vector2d( M_PI,  M_PI);
    }

    Eigen::Vector2d fk(const Eigen::Vector2d &q) const {
        double q1 = q(0), q2 = q(1);
        double x = L1 * std::cos(q1) + L2 * std::cos(q1 + q2);
        double y = L1 * std::sin(q1) + L2 * std::sin(q1 + q2);
        return Eigen::Vector2d(x, y);
    }

    Eigen::Matrix2d jacobian(const Eigen::Vector2d &q) const {
        double q1 = q(0), q2 = q(1);
        double s1 = std::sin(q1);
        double c1 = std::cos(q1);
        double s12 = std::sin(q1 + q2);
        double c12 = std::cos(q1 + q2);

        Eigen::Matrix2d J;
        J(0,0) = -L1 * s1 - L2 * s12;
        J(0,1) = -L2 * s12;
        J(1,0) =  L1 * c1 + L2 * c12;
        J(1,1) =  L2 * c12;
        return J;
    }

    Eigen::Vector2d clip(const Eigen::Vector2d &q) const {
        Eigen::Vector2d qc;
        for (int i = 0; i < 2; ++i) {
            qc(i) = std::min(std::max(q(i), q_min(i)), q_max(i));
        }
        return qc;
    }
};

struct IKResult {
    Eigen::Vector2d q_star;
    bool success;
    int iters;
};

IKResult ik_dls(const Planar2RModel &model,
                const Eigen::Vector2d &x_des,
                const Eigen::Vector2d &q0,
                double lambda_init = 1e-2,
                double eps_task = 1e-4,
                int max_iters = 100,
                double step_max = 0.2)
{
    Eigen::Vector2d q = model.clip(q0);
    double lambda = lambda_init;

    for (int k = 0; k < max_iters; ++k) {
        Eigen::Vector2d x = model.fk(q);
        Eigen::Vector2d e = x_des - x;
        double err = e.norm();

        if (err <= eps_task) {
            return {q, true, k};
        }

        Eigen::Matrix2d J = model.jacobian(q);
        Eigen::Matrix2d H = J.transpose() * J
                            + lambda * lambda * Eigen::Matrix2d::Identity();
        Eigen::Vector2d g = J.transpose() * e;

        Eigen::Vector2d delta_q = H.ldlt().solve(g);
        double step_norm = delta_q.norm();
        if (step_norm > step_max) {
            delta_q *= (step_max / step_norm);
        }

        Eigen::Vector2d q_new = model.clip(q + delta_q);
        if ((q_new - (q + delta_q)).norm() > 1e-6) {
            lambda *= 2.0;
        } else {
            Eigen::Vector2d x_new = model.fk(q_new);
            if (x_new - x_des).norm() < err {
                lambda = std::max(lambda * 0.7, 1e-4);
            }
        }
        q = q_new;
    }
    return {q, false, max_iters};
}

int main() {
    Planar2RModel model(0.5, 0.4);
    Eigen::Vector2d x_des(0.6, 0.3);

    // Noisy target
    Eigen::Vector2d noise;
    noise.setRandom();
    noise *= 0.005;
    Eigen::Vector2d x_noisy = x_des + noise;

    Eigen::Vector2d q0(0.0, 0.0);
    IKResult res = ik_dls(model, x_noisy, q0);

    std::cout << "Success: " << std::boolalpha << res.success << "\n";
    std::cout << "q_star: " << res.q_star.transpose() << "\n";
    std::cout << "achieved x: " << model.fk(res.q_star).transpose() << "\n";
    return 0;
}
      
