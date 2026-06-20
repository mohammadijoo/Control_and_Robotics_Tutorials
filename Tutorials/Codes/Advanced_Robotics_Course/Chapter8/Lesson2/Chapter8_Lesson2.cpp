#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

using Vec3 = Eigen::Vector3d;

inline Vec3 unit(const Vec3& v) {
    double n = v.norm();
    if (n == 0.0) {
        throw std::runtime_error("Zero vector cannot be normalized.");
    }
    return v / n;
}

double angle_between(const Vec3& a, const Vec3& b) {
    Vec3 au = unit(a);
    Vec3 bu = unit(b);
    double cos_th = au.dot(bu);
    if (cos_th > 1.0) cos_th = 1.0;
    if (cos_th < -1.0) cos_th = -1.0;
    return std::acos(cos_th);
}

bool isAntipodal(
    const Vec3& p1,
    const Vec3& n1_out,
    const Vec3& p2,
    const Vec3& n2_out,
    double mu,
    double tol = 1e-3
) {
    Vec3 n1 = unit(n1_out);
    Vec3 n2 = unit(n2_out);

    Vec3 d = unit(p2 - p1);
    double phi = std::atan(mu);

    double theta1 = angle_between(-n1, d);
    double theta2 = angle_between(n2, -d);
    double theta_n = angle_between(n1, -n2);

    return (theta1 <= phi + tol)
        && (theta2 <= phi + tol)
        && (theta_n >= M_PI - 0.3);
}

// Example integration in a ROS/MoveIt node:
// - Subscribe to a point cloud or mesh.
// - Compute candidate contacts and normals.
// - Call isAntipodal(p1, n1, p2, n2, mu) to filter pairs.
// - Pass surviving grasps to MoveIt as grasp candidates.
      
