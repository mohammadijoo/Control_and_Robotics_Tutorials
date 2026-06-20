#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class TwoLinkCollocationNLP : public Ipopt::TNLP {
public:
    TwoLinkCollocationNLP(int N_in, double T_in)
    : N(N_in), T(T_in), h(T_in / N_in), nx(4), nu(2)
    {
        // total variables: (N+1)*nx + (N+1)*nu
        n_var = (N + 1) * (nx + nu);
        n_con_eq = N * nx + 2 * nx; // collocation + boundary
        // inequality bounds encoded via variable bounds
    }

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m,
                              Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag,
                              Ipopt::TNLP::IndexStyleEnum& index_style) override {
        n = n_var;
        m = n_con_eq;
        // Sparse pattern sizes (here we give simple dense estimates)
        nnz_jac_g = n * m;
        nnz_h_lag = n * n;
        index_style = Ipopt::TNLP::C_STYLE;
        return true;
    }

    virtual bool get_bounds_info(Ipopt::Index n, double* x_l, double* x_u,
                                 Ipopt::Index m, double* g_l, double* g_u) override {
        // Variable bounds: q, v, u
        const double q_min = -M_PI;
        const double q_max =  M_PI;
        const double v_max =  4.0;
        const double u_max = 10.0;

        for (int k = 0; k <= N; ++k) {
            int offset = k * (nx + nu);
            // q1, q2
            x_l[offset + 0] = q_min; x_u[offset + 0] = q_max;
            x_l[offset + 1] = q_min; x_u[offset + 1] = q_max;
            // v1, v2
            x_l[offset + 2] = -v_max; x_u[offset + 2] = v_max;
            x_l[offset + 3] = -v_max; x_u[offset + 3] = v_max;
            // u1, u2
            x_l[offset + 4] = -u_max; x_u[offset + 4] = u_max;
            x_l[offset + 5] = -u_max; x_u[offset + 5] = u_max;
        }

        // Equality constraints: collocation and boundary
        for (int i = 0; i < m; ++i) {
            g_l[i] = 0.0;
            g_u[i] = 0.0;
        }
        return true;
    }

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, double* x,
                                    bool init_z, double* z_L, double* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    double* lambda) override {
        if (!init_x) return false;
        // initialize all states at x_init and controls at zero
        VectorXd x_init(nx);
        x_init << 0.0, 0.0, 0.0, 0.0;
        for (int k = 0; k <= N; ++k) {
            int offset = k * (nx + nu);
            for (int i = 0; i < nx; ++i) x[offset + i] = x_init[i];
            x[offset + 4] = 0.0; // u1
            x[offset + 5] = 0.0; // u2
        }
        return true;
    }

    virtual bool eval_f(Ipopt::Index n, const double* x, bool new_x,
                        double& obj_value) override {
        // Quadratic torque cost approximation
        obj_value = 0.0;
        for (int k = 0; k <= N; ++k) {
            int offset = k * (nx + nu);
            double u1 = x[offset + 4];
            double u2 = x[offset + 5];
            obj_value += 0.5 * h * (u1*u1 + u2*u2);
        }
        return true;
    }

    virtual bool eval_g(Ipopt::Index n, const double* x, bool new_x,
                        Ipopt::Index m, double* g) override {
        // Boundary constraints: x_0 and x_N
        VectorXd x_init(nx), x_goal(nx);
        x_init << 0.0, 0.0, 0.0, 0.0;
        x_goal << M_PI / 2.0, 0.0, 0.0, 0.0;

        int row = 0;
        // x_0 - x_init = 0
        for (int i = 0; i < nx; ++i) {
            g[row++] = x[i] - x_init[i];
        }

        // x_N - x_goal = 0
        int offsetN = N * (nx + nu);
        for (int i = 0; i < nx; ++i) {
            g[row++] = x[offsetN + i] - x_goal[i];
        }

        // Collocation constraints
        for (int k = 0; k < N; ++k) {
            int offk  = k * (nx + nu);
            int offk1 = (k + 1) * (nx + nu);

            Eigen::Map<const VectorXd> xk(&x[offk], nx);
            Eigen::Map<const VectorXd> xk1(&x[offk1], nx);

            Eigen::Vector2d uk(x[offk + 4],  x[offk + 5]);
            Eigen::Vector2d uk1(x[offk1 + 4], x[offk1 + 5]);

            VectorXd fk  = two_link_dynamics(xk,  uk);
            VectorXd fk1 = two_link_dynamics(xk1, uk1);

            VectorXd defect = xk1 - xk - 0.5 * h * (fk + fk1);
            for (int i = 0; i < nx; ++i) {
                g[row++] = defect[i];
            }
        }
        return true;
    }

    // eval_grad_f, eval_jac_g, eval_h and other methods must be implemented.

private:
    int N;
    double T, h;
    int nx, nu;
    int n_var, n_con_eq;

    VectorXd two_link_dynamics(const VectorXd& x, const Eigen::Vector2d& u) const {
        // Same equations as in Python version, but using Eigen.
        // Returns xdot = [v1, v2, a1, a2]
        // ...
        VectorXd xdot(4);
        // fill xdot here
        return xdot;
    }
};
      
