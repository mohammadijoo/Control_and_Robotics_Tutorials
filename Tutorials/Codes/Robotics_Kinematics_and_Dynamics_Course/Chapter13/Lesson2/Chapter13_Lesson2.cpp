#include <Eigen/Dense>

using Matrix3 = Eigen::Matrix<double, 3, 3>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

inline Matrix3 skew(const Vector3& v) {
    Matrix3 S;
    S << 0.0,   -v(2),  v(1),
          v(2),  0.0,   -v(0),
         -v(1),  v(0),  0.0;
    return S;
}

struct SpatialInertia {
    double  m;   // mass
    Vector3 c;   // O to CoM
    Matrix3 Ic;  // inertia about CoM, expressed in O's axes

    Matrix6 matrix() const {
        Matrix3 S = skew(c);
        Matrix3 I3 = Matrix3::Identity();

        Matrix3 I11 = Ic - m * (S * S); // or Ic + m * S * S.transpose()
        Matrix3 I12 = m * S;
        Matrix3 I21 = -m * S;
        Matrix3 I22 = m * I3;

        Matrix6 Is;
        Is.setZero();
        Is.block<3,3>(0,0) = I11;
        Is.block<3,3>(0,3) = I12;
        Is.block<3,3>(3,0) = I21;
        Is.block<3,3>(3,3) = I22;
        return Is;
    }

    Vector6 momentum(const Vector6& v) const {
        return matrix() * v;
    }

    double kineticEnergy(const Vector6& v) const {
        Vector6 h = momentum(v);
        return 0.5 * v.dot(h);
    }
};

// Example usage
int main() {
    SpatialInertia I;
    I.m = 5.0;
    I.c = Vector3(0.0, 0.0, 0.2);
    I.Ic = Matrix3::Zero();
    I.Ic.diagonal() << 0.1, 0.2, 0.15;

    Matrix6 Is = I.matrix();
    Vector6 v;
    v << 0.0, 0.0, 1.0, 0.1, 0.0, 0.0;

    Vector6 h = I.momentum(v);
    double T = I.kineticEnergy(v);
    std::cout << "Is =\n" << Is << std::endl;
    std::cout << "h = " << h.transpose() << std::endl;
    std::cout << "T = " << T << std::endl;
    return 0;
}
      
