#include <iostream>
#include <Eigen/Dense>

using Quaternion = Eigen::Quaterniond;
using Vec3 = Eigen::Vector3d;

Quaternion quatFromAxisAngle(const Vec3 &axis, double theta) {
    Vec3 u = axis.normalized();
    double half = 0.5 * theta;
    double s = std::sin(half);
    return Quaternion(std::cos(half), u.x() * s, u.y() * s, u.z() * s);
}

Quaternion quatSlerp(const Quaternion &q0_in,
                     const Quaternion &q1_in,
                     double t) {
    Quaternion q0 = q0_in.normalized();
    Quaternion q1 = q1_in.normalized();

    double dot = q0.w()*q1.w() + q0.vec().dot(q1.vec());

    // Ensure shortest path
    if (dot < 0.0) {
        q1.coeffs() = -q1.coeffs();
        dot = -dot;
    }

    dot = std::max(-1.0, std::min(1.0, dot));
    double theta = std::acos(dot);

    if (theta < 1e-6) {
        Quaternion q = Quaternion(
            (1.0 - t) * q0.w() + t * q1.w(),
            (1.0 - t) * q0.x() + t * q1.x(),
            (1.0 - t) * q0.y() + t * q1.y(),
            (1.0 - t) * q0.z() + t * q1.z()
        );
        return q.normalized();
    }

    double sin_theta = std::sin(theta);
    double w0 = std::sin((1.0 - t) * theta) / sin_theta;
    double w1 = std::sin(t * theta) / sin_theta;

    return Quaternion(
        w0 * q0.w() + w1 * q1.w(),
        w0 * q0.x() + w1 * q1.x(),
        w0 * q0.y() + w1 * q1.y(),
        w0 * q0.z() + w1 * q1.z()
    ).normalized();
}

int main() {
    Quaternion q0 = quatFromAxisAngle(Vec3(0, 0, 1), 0.0);
    Quaternion q1 = quatFromAxisAngle(Vec3(0, 0, 1), M_PI / 2.0);
    double t = 0.5;
    Quaternion qm = quatSlerp(q0, q1, t);

    Vec3 v(1.0, 0.0, 0.0);
    Vec3 v_rot = qm * v;  // Eigen applies rotation via q * v * q^{-1}

    std::cout << "qm = " << qm.coeffs().transpose() << std::endl;
    std::cout << "v_rot = " << v_rot.transpose() << std::endl;
    return 0;
}
      
