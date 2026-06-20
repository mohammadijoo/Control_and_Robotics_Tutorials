#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;

Mat3 skew(const Vec3& p) {
    Mat3 S;
    S << 0.0,   -p.z(),  p.y(),
          p.z(),  0.0,   -p.x(),
         -p.y(),  p.x(),  0.0;
    return S;
}

Mat63 graspMapBlock(const Vec3& p_i, const Mat3& R_i) {
    Mat63 G_i;
    G_i.block<3,3>(0,0) = R_i;
    G_i.block<3,3>(3,0) = skew(p_i) * R_i;
    return G_i;
}

Eigen::MatrixXd linearizedFrictionDirections(double mu, int k_dirs = 8) {
    Eigen::MatrixXd D(3, k_dirs);
    double alpha = std::atan(mu);
    double fn = std::cos(alpha);
    double ft = std::sin(alpha);
    for (int j = 0; j < k_dirs; ++j) {
        double theta = 2.0 * M_PI * j / static_cast<double>(k_dirs);
        double tx = ft * std::cos(theta);
        double ty = ft * std::sin(theta);
        D.col(j) = Vec3(tx, ty, fn);
    }
    return D;
}

Mat6X primitiveWrenches(const std::vector<Vec3>& p_list,
                        const std::vector<Mat3>& R_list,
                        const std::vector<double>& mu_list,
                        int k_dirs = 8)
{
    int m = static_cast<int>(p_list.size());
    int N = m * k_dirs;
    Mat6X W(6, N);
    int col = 0;
    for (int i = 0; i < m; ++i) {
        Mat63 G_i = graspMapBlock(p_list[i], R_list[i]);
        Eigen::MatrixXd D_i = linearizedFrictionDirections(mu_list[i], k_dirs);
        Mat6X W_i = G_i * D_i;
        for (int j = 0; j < k_dirs; ++j) {
            W.col(col++) = W_i.col(j);
        }
    }
    return W;
}

double isotropyMetric(const Mat6X& G) {
    Eigen::JacobiSVD<Mat6X> svd(G, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    double s_min = s.minCoeff();
    double s_max = s.maxCoeff();
    if (s_max < 1e-12) return 0.0;
    return s_min / s_max;
}

int main() {
    // Example: trivial 3-contact setup (placeholder geometry)
    std::vector<Vec3> p_list;
    std::vector<Mat3> R_list;
    std::vector<double> mu_list;

    p_list.emplace_back(0.05, 0.0, 0.0);
    p_list.emplace_back(-0.025, 0.043301, 0.0);
    p_list.emplace_back(-0.025, -0.043301, 0.0);

    for (const auto& p : p_list) {
        Vec3 n = -p.normalized();
        Vec3 tmp(1.0, 0.0, 0.0);
        if (std::abs(tmp.dot(n)) > 0.9) {
            tmp = Vec3(0.0, 1.0, 0.0);
        }
        Vec3 x = (tmp - tmp.dot(n) * n).normalized();
        Vec3 y = n.cross(x);
        Mat3 R;
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = n;
        R_list.push_back(R);
        mu_list.push_back(0.8);
    }

    Mat6X W = primitiveWrenches(p_list, R_list, mu_list, 8);
    double q_iso = isotropyMetric(W);

    std::cout << "Isotropy metric Q_iso = " << q_iso << std::endl;
    return 0;
}
      
