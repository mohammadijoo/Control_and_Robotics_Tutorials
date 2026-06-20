#include <iostream>
#include <Eigen/Dense>

using Quaterniond = Eigen::Quaterniond;
using Matrix3d    = Eigen::Matrix3d;
using Vector3d    = Eigen::Vector3d;

Quaterniond continuousQuat(const Quaterniond& q_prev,
                           const Quaterniond& q_raw)
{
    Quaterniond q_new = q_raw;
    if (q_prev.coeffs().dot(q_new.coeffs()) < 0.0)
    {
        q_new.coeffs() *= -1.0;
    }
    q_new.normalize();
    return q_new;
}

Vector3d rotmToEulerZYX(const Matrix3d& R, bool& nearGimbal,
                        double eps = 1e-6)
{
    double theta = -std::asin(R(2, 0));
    double cosTheta = std::cos(theta);
    nearGimbal = std::abs(cosTheta) < eps;

    double phi, psi;
    if (!nearGimbal)
    {
        phi = std::atan2(R(1, 0), R(0, 0)); // yaw
        psi = std::atan2(R(2, 1), R(2, 2)); // roll
    }
    else
    {
        phi = 0.0;
        if (theta > 0.0)
            psi = std::atan2(R(0, 1), R(1, 1));
        else
            psi = -std::atan2(R(0, 1), R(1, 1));
    }
    return Vector3d(phi, theta, psi);
}

int main()
{
    Matrix3d R = Matrix3d::Identity();
    Quaterniond q_prev(1.0, 0.0, 0.0, 0.0);
    Quaterniond q_raw(R);
    Quaterniond q_cont = continuousQuat(q_prev, q_raw);

    bool nearGimbal = false;
    Vector3d eul = rotmToEulerZYX(R, nearGimbal);
    std::cout << "q_cont = " << q_cont.coeffs().transpose() << std::endl;
    std::cout << "Euler (phi, theta, psi) = " << eul.transpose()
              << ", nearGimbal = " << nearGimbal << std::endl;
    return 0;
}
      
