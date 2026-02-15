#include <Eigen/Dense>
#include <vector>

using SE3 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Twist = Eigen::Matrix<double, 6, 1>;

Eigen::Matrix3d skew3(const Vec3& w) {
    Eigen::Matrix3d W;
    W << 0.0,   -w.z(),  w.y(),
          w.z(),  0.0,   -w.x(),
         -w.y(),  w.x(),  0.0;
    return W;
}

SE3 expTwist(const Twist& xi, double q, double eps = 1e-9) {
    Vec3 w = xi.head<3>();
    Vec3 v = xi.tail<3>();

    SE3 T = SE3::Identity();
    double theta = q;
    if (w.norm() < eps) {
        // prismatic
        T.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        T.block<3,1>(0,3) = v * theta;
    } else {
        w.normalize();
        Eigen::Matrix3d W = skew3(w);
        Eigen::Matrix3d W2 = W * W;
        Eigen::Matrix3d R =
            Eigen::Matrix3d::Identity()
            + std::sin(theta) * W
            + (1.0 - std::cos(theta)) * W2;
        Eigen::Matrix3d V =
            Eigen::Matrix3d::Identity() * theta
            + (1.0 - std::cos(theta)) * W
            + (theta - std::sin(theta)) * W2;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = V * v;
    }
    return T;
}

struct RobotPoeCpp {
    std::vector<Twist> S_list;
    SE3 M;

    SE3 fk(const std::vector<double>& q) const {
        SE3 T = SE3::Identity();
        for (std::size_t i = 0; i < S_list.size(); ++i) {
            T = T * expTwist(S_list[i], q[i]);
        }
        return T * M;
    }
};
      
