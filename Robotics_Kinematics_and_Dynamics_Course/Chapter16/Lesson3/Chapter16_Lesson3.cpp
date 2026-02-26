#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::JacobiSVD;

struct SingularityInfo {
    std::string type;
    double detA;
    double detB;
    int rankA;
    int rankB;
    double kappaA;
    double kappaB;
};

SingularityInfo classifySingularity(const Matrix3d& A,
                                    const Matrix3d& B,
                                    double tol = 1e-6)
{
    SingularityInfo info;

    info.detA = A.determinant();
    info.detB = B.determinant();

    JacobiSVD<Matrix3d> svdA(A);
    JacobiSVD<Matrix3d> svdB(B);

    Eigen::Vector3d sA = svdA.singularValues();
    Eigen::Vector3d sB = svdB.singularValues();

    info.rankA = (sA.array() > tol).count();
    info.rankB = (sB.array() > tol).count();

    info.kappaA = (sA(2) > tol) ? sA(0) / sA(2) : std::numeric_limits<double>::infinity();
    info.kappaB = (sB(2) > tol) ? sB(0) / sB(2) : std::numeric_limits<double>::infinity();

    int m = 3;
    bool typeI  = info.rankB < m;
    bool typeII = info.rankA < m;

    if (typeI && typeII) {
        info.type = "Type III (combined)";
    } else if (typeI) {
        info.type = "Type I (serial)";
    } else if (typeII) {
        info.type = "Type II (parallel)";
    } else {
        info.type = "Regular";
    }

    return info;
}

int main() {
    Matrix3d A, B;
    // Fill A and B here (e.g., using a function similar to compute_AB in Python)

    // Dummy example:
    A.setIdentity();
    B.setIdentity();

    SingularityInfo info = classifySingularity(A, B);

    std::cout << "Type: " << info.type << std::endl;
    std::cout << "det(A) = " << info.detA
              << ", det(B) = " << info.detB << std::endl;
    std::cout << "rank(A) = " << info.rankA
              << ", rank(B) = " << info.rankB << std::endl;
    std::cout << "kappa(A) = " << info.kappaA
              << ", kappa(B) = " << info.kappaB << std::endl;

    return 0;
}
      
