
#include <Eigen/Dense>
#include <vector>
#include <iostream>

// Each regressor sample is an Eigen::VectorXd of size m
double computeMinEigenvalue(
    const std::vector<Eigen::VectorXd> &samples,
    double dt)
{
    if (samples.empty()) {
        return 0.0;
    }
    std::size_t m = static_cast<std::size_t>(samples[0].size());
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(m, m);
    for (std::size_t k = 0; k < samples.size(); ++k) {
        const Eigen::VectorXd &phi = samples[k];
        G += phi * phi.transpose();
    }
    G *= dt;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(G);
    if (es.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed");
    }
    Eigen::VectorXd vals = es.eigenvalues();
    return vals.minCoeff();
}

int main() {
    // Example: suppose we already filled samples with regressor vectors
    std::vector<Eigen::VectorXd> samples;
    double dt = 0.001;

    // ... fill samples using a robotics dynamics library:
    // for each time step, call something like:
    //   RBDL::NonlinearEffects(model, q, qdot, tau);
    // or build a regressor Y(q, qdot, qr, qrdot) manually.

    try {
        double lambda_min = computeMinEigenvalue(samples, dt);
        std::cout << "lambda_min(G_N) = " << lambda_min << std::endl;
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }
    return 0;
}
