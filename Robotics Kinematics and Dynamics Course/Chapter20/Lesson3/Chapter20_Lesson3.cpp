#include <iostream>
#include <Eigen/Dense>
#include <random>

int main() {
    using Eigen::Matrix2d;
    using Eigen::Vector2d;
    using Eigen::RowVector2d;

    // Parameter distribution
    Vector2d theta_mean;
    theta_mean << 1.5, 0.8;

    Matrix2d Sigma_theta;
    Sigma_theta <<
        0.04, 0.0,
        0.0,  0.01;

    // Regressor Y(q, qd, qdd)
    double q   = 0.5;
    double qd  = 0.0;
    double qdd = 2.0;
    RowVector2d Y;
    Y << qdd, std::sin(q);

    // Analytical mean and variance of tau
    double tau_mean = (Y * theta_mean)(0);
    double tau_var  = (Y * Sigma_theta * Y.transpose())(0, 0);

    std::cout << "Analytical E[tau] = " << tau_mean << std::endl;
    std::cout << "Analytical Var[tau] = " << tau_var << std::endl;

    // Monte Carlo simulation
    std::mt19937 gen(0);
    std::normal_distribution<double> std_normal(0.0, 1.0);

    // Cholesky of Sigma_theta (since it is SPD)
    Eigen::LLT<Matrix2d> llt(Sigma_theta);
    Matrix2d L = llt.matrixL();

    const int Nmc = 100000;
    double mean_mc = 0.0;
    double m2_mc   = 0.0;

    for (int k = 0; k < Nmc; ++k) {
        // Sample z ~ N(0, I)
        Vector2d z;
        z << std_normal(gen), std_normal(gen);

        // theta_sample = theta_mean + L * z
        Vector2d theta_sample = theta_mean + L * z;

        double tau_sample = (Y * theta_sample)(0);

        // Online mean/variance (Welford)
        double delta = tau_sample - mean_mc;
        mean_mc += delta / static_cast<double>(k + 1);
        m2_mc   += delta * (tau_sample - mean_mc);
    }

    double var_mc = m2_mc / static_cast<double>(Nmc - 1);

    std::cout << "MC mean approx = " << mean_mc << std::endl;
    std::cout << "MC var approx  = " << var_mc << std::endl;

    return 0;
}
      
