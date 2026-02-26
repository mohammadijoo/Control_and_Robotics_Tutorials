#include <Eigen/Dense>
#include <array>

class StewartPlatform {
public:
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    StewartPlatform(const std::array<Vec3, 6>& basePoints,
                    const std::array<Vec3, 6>& platformPoints)
        : B_(basePoints), P_(platformPoints) {}

    static Mat3 rotZYX(double phi, double theta, double psi) {
        double cphi = std::cos(phi), sphi = std::sin(phi);
        double cth  = std::cos(theta), sth  = std::sin(theta);
        double cpsi = std::cos(psi), spsi = std::sin(psi);

        Mat3 Rx, Ry, Rz;
        Rx << 1.0, 0.0, 0.0,
               0.0, cphi, -sphi,
               0.0, sphi, cphi;
        Ry << cth, 0.0, sth,
               0.0, 1.0, 0.0,
               -sth, 0.0, cth;
        Rz << cpsi, -spsi, 0.0,
               spsi, cpsi, 0.0,
               0.0, 0.0, 1.0;
        return Rz * Ry * Rx;
    }

    // IK: pose = [px, py, pz, phi, theta, psi]
    Eigen::Matrix<double, 6, 1> inverseKinematics(const Eigen::Matrix<double, 6, 1>& pose) const {
        double px = pose(0), py = pose(1), pz = pose(2);
        double phi = pose(3), theta = pose(4), psi = pose(5);
        Vec3 p(px, py, pz);
        Mat3 R = rotZYX(phi, theta, psi);

        Eigen::Matrix<double, 6, 1> L;
        for (int i = 0; i < 6; ++i) {
            Vec3 d = p + R * P_[i] - B_[i];
            L(i) = d.norm();
        }
        return L;
    }

    // Length-rate Jacobian JL
    Mat6 jacobianLengthRate(const Eigen::Matrix<double, 6, 1>& pose) const {
        double px = pose(0), py = pose(1), pz = pose(2);
        double phi = pose(3), theta = pose(4), psi = pose(5);
        Vec3 p(px, py, pz);
        Mat3 R = rotZYX(phi, theta, psi);

        Mat6 JL;
        JL.setZero();

        for (int i = 0; i < 6; ++i) {
            Vec3 d = p + R * P_[i] - B_[i];
            double L = d.norm();
            Vec3 u = d / L;
            Vec3 Rp = R * P_[i];
            Vec3 cross = Rp.cross(u);

            JL.block<1,3>(i, 0) = u.transpose();
            JL.block<1,3>(i, 3) = cross.transpose();
        }
        return JL;
    }

private:
    std::array<Vec3, 6> B_;  // base points
    std::array<Vec3, 6> P_;  // platform points
};
      
