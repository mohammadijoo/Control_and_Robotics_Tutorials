#pragma once
#include <Eigen/Dense>

struct RigidBodyInertia {
    double m;                // mass
    Eigen::Vector3d com;     // center of mass expressed in frame B
    Eigen::Matrix3d I_com;   // inertia about CoM, expressed in frame B

    RigidBodyInertia()
        : m(0.0), com(Eigen::Vector3d::Zero()), I_com(Eigen::Matrix3d::Zero()) {}

    RigidBodyInertia(double mass,
                     const Eigen::Vector3d &com_B,
                     const Eigen::Matrix3d &I_com_B)
        : m(mass), com(com_B), I_com(I_com_B) {}

    Eigen::Matrix3d inertiaAboutPoint(const Eigen::Vector3d &p_B) const {
        // Vector from reference point P (coords p_B) to CoM
        Eigen::Vector3d c_rel = com - p_B;
        double c2 = c_rel.squaredNorm();
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        return I_com + m * (c2 * I3 - c_rel * c_rel.transpose());
    }

    Eigen::Matrix3d inertiaAboutOrigin() const {
        return inertiaAboutPoint(Eigen::Vector3d::Zero());
    }

    Eigen::Matrix<double, 10, 1> parameterVectorAboutOrigin() const {
        Eigen::Matrix3d I_B = inertiaAboutOrigin();
        double Ixx = I_B(0, 0);
        double Iyy = I_B(1, 1);
        double Izz = I_B(2, 2);
        double Ixy = -I_B(0, 1);
        double Ixz = -I_B(0, 2);
        double Iyz = -I_B(1, 2);

        Eigen::Matrix<double, 10, 1> pi;
        pi << m,
              m * com.x(), m * com.y(), m * com.z(),
              Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        return pi;
    }

    RigidBodyInertia rotated(const Eigen::Matrix3d &R_B_Bp) const {
        // R_B_Bp maps coordinates from B' to B: v_B = R_B_Bp * v_Bp
        Eigen::Vector3d com_Bp = R_B_Bp.transpose() * com;
        Eigen::Matrix3d I_com_Bp = R_B_Bp.transpose() * I_com * R_B_Bp;
        return RigidBodyInertia(m, com_Bp, I_com_Bp);
    }
};

// Example: build a box inertia (uniform density)
inline RigidBodyInertia makeUniformBox(double m,
                                       double lx, double ly, double lz)
{
    // box centered at origin with dimensions lx, ly, lz along x, y, z
    Eigen::Vector3d com_B(0.0, 0.0, 0.0);
    double Ixx = (1.0 / 12.0) * m * (ly * ly + lz * lz);
    double Iyy = (1.0 / 12.0) * m * (lx * lx + lz * lz);
    double Izz = (1.0 / 12.0) * m * (lx * lx + ly * ly);

    Eigen::Matrix3d I_com_B = Eigen::Matrix3d::Zero();
    I_com_B(0, 0) = Ixx;
    I_com_B(1, 1) = Iyy;
    I_com_B(2, 2) = Izz;

    return RigidBodyInertia(m, com_B, I_com_B);
}
      
