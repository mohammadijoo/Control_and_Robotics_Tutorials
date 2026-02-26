#include <iostream>
#include <Eigen/Dense>

int main() {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const int N = 50;   // number of samples
    const int p = 4;    // number of full parameters in this example

    MatrixXd W(N, p);

    // TODO: fill W with regressor rows from your dynamics library.
    // Here we simply fake something with random values to illustrate:
    W.setRandom();

    // Compute thin SVD: W = U * S * V^T
    Eigen::JacobiSVD<MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorXd S = svd.singularValues();
    MatrixXd V = svd.matrixV();

    double tol = 1e-8;
    int r = 0;
    for (int i = 0; i < S.size(); ++i) {
        if (S(i) > tol) { ++r; }
    }
    std::cout << "Rank r = " << r << std::endl;

    MatrixXd V1 = V.leftCols(r);         // base parameter subspace
    MatrixXd V2 = V.rightCols(p - r);    // null-space basis

    // Example full parameter vector
    VectorXd pi_full(p);
    pi_full << 0.1, 2.0, 0.5, 1.0;

    // Base parameters beta = V1^T * pi
    VectorXd beta = V1.transpose() * pi_full;

    std::cout << "beta =\n" << beta << std::endl;
    std::cout << "Null-space basis V2 =\n" << V2 << std::endl;

    return 0;
}
      
