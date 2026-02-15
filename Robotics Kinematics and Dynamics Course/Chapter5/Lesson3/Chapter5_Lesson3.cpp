#include <Eigen/Dense>
#include <vector>

using Mat4 = Eigen::Matrix4d;
using Vec  = Eigen::VectorXd;

Mat4 dhTransform(double theta, double d, double a, double alpha)
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Mat4 A;
    A <<
        ct, -st * ca,  st * sa, a * ct,
        st,  ct * ca, -ct * sa, a * st,
        0.0,      sa,      ca,       d,
        0.0,     0.0,     0.0,     1.0;
    return A;
}

Mat4 fkDH(const Vec& q,
          const Vec& a,
          const Vec& alpha,
          const Vec& d,
          const Vec& thetaOffset)
{
    const int n = static_cast<int>(q.size());
    Mat4 T = Mat4::Identity();
    for (int i = 0; i < n; ++i)
    {
        double theta_i = q(i) + thetaOffset(i);
        Mat4 A_i = dhTransform(theta_i, d(i), a(i), alpha(i));
        T = T * A_i;
    }
    return T;
}

// Example: planar 2R FK
Mat4 fkPlanar2R(double q1, double q2, double l1, double l2)
{
    Vec q(2);      q << q1, q2;
    Vec a(2);      a << l1, l2;
    Vec alpha(2);  alpha.setZero();
    Vec d(2);      d.setZero();
    Vec thetaOff(2); thetaOff.setZero();
    return fkDH(q, a, alpha, d, thetaOff);
}
      
