#include <iostream>
#include <fstream>
#include <Eigen/Dense>

// Load Phi (N x (d+1)) and A (N x m) from plain-text files
bool loadMatrix(const std::string& path, Eigen::MatrixXf& M, int rows, int cols) {
    std::ifstream in(path);
    if (!in) return false;
    M.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            in >> M(i, j);
        }
    }
    return true;
}

int main() {
    int N = 10000;     // number of samples
    int d1 = 33;       // feature dimension + 1 for bias
    int m  = 7;        // number of joints

    Eigen::MatrixXf Phi_tilde;
    Eigen::MatrixXf A;
    if (!loadMatrix("Phi_tilde.txt", Phi_tilde, N, d1)) {
        std::cerr << "Could not load Phi_tilde\n";
        return 1;
    }
    if (!loadMatrix("A.txt", A, N, m)) {
        std::cerr << "Could not load A\n";
        return 1;
    }

    float lambda = 1e-3f;

    // Compute W_tilde* = (Phi^T Phi + N lambda I)^(-1) Phi^T A
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(d1, d1);
    Eigen::MatrixXf G = Phi_tilde.transpose() * Phi_tilde + N * lambda * I;
    Eigen::MatrixXf RHS = Phi_tilde.transpose() * A;

    Eigen::MatrixXf W_tilde = G.ldlt().solve(RHS);

    // Split into W (m x d) and b (m)
    int d = d1 - 1;
    Eigen::MatrixXf W = W_tilde.leftCols(d);
    Eigen::VectorXf b = W_tilde.col(d);

    // Example: evaluate policy for one state
    Eigen::VectorXf s(d);
    // ... fill s with features ...
    Eigen::VectorXf a_cmd = W * s + b;

    std::cout << "Commanded action:\n" << a_cmd.transpose() << std::endl;
    return 0;
}
      
