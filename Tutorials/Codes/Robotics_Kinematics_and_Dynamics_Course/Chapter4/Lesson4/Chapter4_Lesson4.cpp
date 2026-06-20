#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct Joint {
    int dof;
    std::string name;
    Joint(int f, const std::string& n) : dof(f), name(n) {}
};

class Mechanism {
public:
    Mechanism(int n_links, int lambda_dim)
        : N(n_links), lambda_dim(lambda_dim) {}

    void addJoint(const Joint& j) {
        joints.push_back(j);
    }

    int mobilityGruebler() const {
        int J = static_cast<int>(joints.size());
        int f_sum = 0;
        for (const auto& j : joints) {
            f_sum += j.dof;
        }
        return lambda_dim * (N - 1 - J) + f_sum;
    }

    static int dofFromConstraints(const Eigen::MatrixXd& Jphi, double tol = 1e-9) {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(Jphi);
        int rank = lu.rank();
        int n = static_cast<int>(Jphi.cols());
        return n - rank;
    }

private:
    int N;                  // number of links (including ground)
    int lambda_dim;         // 6 for spatial, 3 for planar
    std::vector<Joint> joints;
};

int main() {
    // Planar 4-bar
    Mechanism fourbar(4, 3);
    for (int i = 0; i < 4; ++i) {
        fourbar.addJoint(Joint(1, "R"));
    }
    std::cout << "4-bar mobility (Gruebler): "
              << fourbar.mobilityGruebler() << std::endl;

    // Rank-based DOF
    Eigen::MatrixXd Jphi(1, 3);
    Jphi << 1.0, 1.0, 1.0;
    int dof = Mechanism::dofFromConstraints(Jphi);
    std::cout << "DOF from constraints: " << dof << std::endl;
    return 0;
}
      
