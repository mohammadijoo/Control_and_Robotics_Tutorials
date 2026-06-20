#include <Eigen/Dense>
#include <random>
#include <iostream>

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

// Time step
const double dt = 0.1;

Vector3 f(const Vector3& x, const Eigen::Vector2d& u)
{
    double px = x(0);
    double py = x(1);
    double phi = x(2);
    double v = u(0);
    double omega = u(1);

    Vector3 xn;
    xn(0) = px + v * std::cos(phi) * dt;
    xn(1) = py + v * std::sin(phi) * dt;
    xn(2) = phi + omega * dt;
    return xn;
}

int main()
{
    // Process noise covariance
    Matrix3 Q = Matrix3::Zero();
    Q(0,0) = 0.01 * 0.01;
    Q(1,1) = 0.01 * 0.01;
    Q(2,2) = std::pow(M_PI / 180.0, 2); // 1 deg in rad

    std::mt19937 gen(0);
    std::normal_distribution<double> standard_normal(0.0, 1.0);

    auto sample_process_noise = [&]() {
        Vector3 z;
        for (int i = 0; i < 3; ++i)
            z(i) = standard_normal(gen);
        // Cholesky factor of Q
        Eigen::LLT<Matrix3> llt(Q);
        Matrix3 L = llt.matrixL();
        return L * z;
    };

    Vector3 x;
    x << 0.0, 0.0, 0.0;
    Eigen::Vector2d u;
    u << 1.0, 0.2;

    const int T = 20;
    for (int t = 0; t < T; ++t) {
        Vector3 x_nom = f(x, u);
        Vector3 w = sample_process_noise();
        x = x_nom + w;
    }

    std::cout << "Final stochastic state: " << x.transpose() << std::endl;

    // In a ROS2 or ROS1 stack, such propagation would often be part of a node
    // using Eigen, tf2, and sensor_msgs for message-passing.
    return 0;
}
      
