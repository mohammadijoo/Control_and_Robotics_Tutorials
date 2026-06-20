
#include <iostream>
#include <Eigen/Dense>

using Vector2 = Eigen::Matrix<double,2,1>;
using Matrix2 = Eigen::Matrix<double,2,2>;

// Nonlinear dynamics: xdot = f(x,u)
Vector2 pendulumDynamics(const Vector2& x,
                         double u,
                         double m, double g, double l, double I)
{
    Vector2 xdot;
    double q     = x(0);
    double qdot  = x(1);

    xdot(0) = qdot;
    xdot(1) = -(m*g*l/I) * std::sin(q) + (1.0/I) * u;
    return xdot;
}

// Finite-difference linearization
void linearizePendulum(const Vector2& x0,
                       double u0,
                       double m, double g, double l, double I,
                       double eps,
                       Matrix2& A,
                       Vector2& B)
{
    Vector2 f0 = pendulumDynamics(x0, u0, m, g, l, I);

    // Columns of A
    for (int i = 0; i < 2; ++i)
    {
        Vector2 x_plus  = x0;
        Vector2 x_minus = x0;
        x_plus(i)  += eps;
        x_minus(i) -= eps;

        Vector2 f_plus  = pendulumDynamics(x_plus,  u0, m, g, l, I);
        Vector2 f_minus = pendulumDynamics(x_minus, u0, m, g, l, I);

        A.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }

    // Column of B (single input)
    double u_plus  = u0 + eps;
    double u_minus = u0 - eps;

    Vector2 f_plus_u  = pendulumDynamics(x0, u_plus,  m, g, l, I);
    Vector2 f_minus_u = pendulumDynamics(x0, u_minus, m, g, l, I);

    B = (f_plus_u - f_minus_u) / (2.0 * eps);
}

int main()
{
    Vector2 x0;
    x0 << 0.0, 0.0;
    double u0 = 0.0;

    double m = 1.0, g = 9.81, l = 1.0, I = 1.0;
    double eps = 1e-6;

    Matrix2 A;
    Vector2 B;
    linearizePendulum(x0, u0, m, g, l, I, eps, A, B);

    std::cout << "A =\n" << A << std::endl;
    std::cout << "B =\n" << B << std::endl;
    return 0;
}
