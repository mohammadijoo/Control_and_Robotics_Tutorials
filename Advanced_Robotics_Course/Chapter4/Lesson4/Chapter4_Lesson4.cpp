#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <Eigen/Dense>

using Eigen::VectorXd;

class DoubleIntegratorCollocationNLP : public Ipopt::TNLP {
public:
    DoubleIntegratorCollocationNLP(int N_in, double T_in)
    : N(N_in), T(T_in), h(T_in / N_in), n_x(2), n_u(1) {}

    virtual bool get_nlp_info(
        Ipopt::Index& n,
        Ipopt::Index& m,
        Ipopt::Index& nnz_jac_g,
        Ipopt::Index& nnz_h_lag,
        IndexStyleEnum& index_style)
    {
        n = (N + 1) * n_x + N * n_u;     // decision variables
        m = 2 * n_x + N * n_x;           // initial, terminal, collocation
        nnz_jac_g = m * n;               // dense for simplicity (sparse in practice)
        nnz_h_lag = 0;                   // let Ipopt approximate Hessian
        index_style = TNLP::C_STYLE;
        return true;
    }

    virtual bool eval_f(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Number& obj_value)
    {
        obj_value = 0.0;
        int offset_u = (N + 1) * n_x;
        for (int k = 0; k < N; ++k) {
            double uk = x[offset_u + k * n_u];
            obj_value += h * uk * uk;
        }
        return true;
    }

    virtual bool eval_g(
        Ipopt::Index n,
        const Ipopt::Number* x,
        bool new_x,
        Ipopt::Index m,
        Ipopt::Number* g)
    {
        int row = 0;

        // Initial state: x_0 = (0, 0)
        for (int i = 0; i < n_x; ++i) {
            g[row++] = x[i] - 0.0;
        }

        // Terminal state: x_N = (1, 0)
        int idx_final = N * n_x;
        g[row++] = x[idx_final] - 1.0;     // position
        g[row++] = x[idx_final + 1] - 0.0; // velocity

        int offset_u = (N + 1) * n_x;

        // Trapezoidal collocation constraints
        for (int k = 0; k < N; ++k) {
            int idx_xk = k * n_x;
            int idx_xkp1 = (k + 1) * n_x;
            double uk = x[offset_u + k * n_u];

            double pk = x[idx_xk];
            double vk = x[idx_xk + 1];
            double pkp1 = x[idx_xkp1];
            double vkp1 = x[idx_xkp1 + 1];

            double fk_p = vk;
            double fk_v = uk;
            double fkp1_p = vkp1;
            double fkp1_v = uk;

            g[row++] = pkp1 - pk - 0.5 * h * (fk_p + fkp1_p);
            g[row++] = vkp1 - vk - 0.5 * h * (fk_v + fkp1_v);
        }
        return true;
    }

    // Other virtual methods (bounds, Jacobian, etc.) omitted for brevity.

private:
    int N;
    int n_x;
    int n_u;
    double T;
    double h;
};

int main()
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    Ipopt::SmartPtr<DoubleIntegratorCollocationNLP> nlp =
        new DoubleIntegratorCollocationNLP(40, 1.0);

    app->Initialize();
    app->OptimizeTNLP(nlp);
    return 0;
}
      
