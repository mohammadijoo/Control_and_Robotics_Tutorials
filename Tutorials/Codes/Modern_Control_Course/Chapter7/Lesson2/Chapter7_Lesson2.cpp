#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using InputFun = std::function<VectorXd(double)>;

std::vector<VectorXd> forcedResponseConvolution(
    const MatrixXd& A,
    const MatrixXd& B,
    const InputFun& u_fun,
    const std::vector<double>& t_grid)
{
    const int n = A.rows();
    std::vector<VectorXd> xzs(t_grid.size(), VectorXd::Zero(n));

    for (size_t k = 1; k < t_grid.size(); ++k)
    {
        double tk = t_grid[k];
        VectorXd acc = VectorXd::Zero(n);

        for (size_t j = 0; j < k; ++j)
        {
            double sj  = t_grid[j];
            double sj1 = t_grid[j+1];
            double dt  = sj1 - sj;

            VectorXd uj  = u_fun(sj);
            VectorXd uj1 = u_fun(sj1);

            MatrixXd K0 = (A * (tk - sj)).exp()  * B;
            MatrixXd K1 = (A * (tk - sj1)).exp() * B;

            acc += 0.5 * (K0 * uj + K1 * uj1) * dt;
        }
        xzs[k] = acc;
    }
    return xzs;
}

std::vector<VectorXd> totalState(
    const MatrixXd& A,
    const VectorXd& x0,
    const std::vector<VectorXd>& xzs,
    const std::vector<double>& t_grid)
{
    std::vector<VectorXd> x(t_grid.size(), x0);
    for (size_t k = 0; k < t_grid.size(); ++k)
    {
        double tk = t_grid[k];
        x[k] = (A * tk).exp() * x0 + xzs[k];
    }
    return x;
}

int main()
{
    MatrixXd A(2,2);
    A << 0.0, 1.0,
        -2.0, -3.0;
    MatrixXd B(2,1);
    B << 0.0,
         1.0;
    VectorXd x0(2);
    x0 << 1.0, 0.0;

    auto u_fun = [](double t) -> VectorXd {
        VectorXd u(1);
        u(0) = (t >= 0.0 && t <= 2.0) ? 1.0 : 0.0; // unit pulse
        return u;
    };

    std::vector<double> t_grid;
    double T = 6.0, dt = 0.01;
    for (int k = 0; k <= static_cast<int>(T/dt); ++k)
        t_grid.push_back(k * dt);

    auto xzs = forcedResponseConvolution(A, B, u_fun, t_grid);
    auto x   = totalState(A, x0, xzs, t_grid);

    std::cout << "x(0) = " << x.front().transpose() << std::endl;
    std::cout << "x(6) = " << x.back().transpose() << std::endl;
    return 0;
}
