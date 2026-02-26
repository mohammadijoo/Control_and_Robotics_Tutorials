#include <Eigen/Dense>
#include <functional>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

// Pose function: R^n -> R^m
using PoseFunction = std::function<Vector(const Vector&)>;

Matrix numericJacobianVec(const PoseFunction& fk,
                          const Vector& q,
                          double h = 1e-6)
{
    Vector qv = q;
    const int n = static_cast<int>(qv.size());
    Vector x0 = fk(qv);
    const int m = static_cast<int>(x0.size());

    Matrix J = Matrix::Zero(m, n);

    for (int i = 0; i < n; ++i) {
        Vector dq = Vector::Zero(n);
        dq(i) = h;

        Vector xp = fk(qv + dq);
        Vector xm = fk(qv - dq);

        J.col(i) = (xp - xm) / (2.0 * h);
    }
    return J;
}

// Example: wrapper around some FK implementation returning a 3D position
Vector fk_example(const Vector& q)
{
    // TODO: call your PoE-based or DH-based FK here
    Vector x(3);
    x.setZero();
    // Fill x based on q
    return x;
}

double checkJacobian(const PoseFunction& fk,
                     const std::function<Matrix(const Vector&)>& jacAna,
                     const Vector& q,
                     double h = 1e-6,
                     double atol = 1e-8)
{
    Matrix J_ana = jacAna(q);
    Matrix J_num = numericJacobianVec(fk, q, h);
    Matrix diff  = J_num - J_ana;

    double err = diff.norm();           // Frobenius for MatrixXd
    double ref = J_ana.norm() + atol;
    return err / ref;
}
      
