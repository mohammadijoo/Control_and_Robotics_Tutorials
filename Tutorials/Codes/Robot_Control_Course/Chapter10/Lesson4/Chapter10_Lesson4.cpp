
#include <Eigen/Dense>
#include <osqp.h>

// Assuming H, G, and other matrices have been constructed similarly to Python code

int main() {
    const int nx = 2;
    const int nu = 1;
    const int N  = 20;
    const int nU = N * nu;
    const int nIneq = 2 * nU;  // simple bounds

    Eigen::MatrixXd H(nU, nU);
    Eigen::MatrixXd F(nU, nx);
    Eigen::MatrixXd G(nIneq, nU);
    Eigen::VectorXd g(nIneq);

    // ... fill H, F, G, g ...

    // Convert to CSC format for OSQP
    c_int n = nU;
    c_int m = nIneq;

    // Build OSQP data structures (P, A, q, l, u)
    // For brevity, details of CSC conversion are omitted

    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    osqp_set_default_settings(settings);
    settings->verbose = 0;
    settings->warm_start = 1;

    // fill data->n, data->m, data->P, data->q, data->A, data->l, data->u
    // ...

    OSQPWorkspace *work = osqp_setup(data, settings);

    Eigen::VectorXd x(nx);
    x << 0.5, 0.0;

    Eigen::VectorXd U_prev = Eigen::VectorXd::Zero(nU);
    Eigen::VectorXd lam_prev = Eigen::VectorXd::Zero(m);

    for (int k = 0; k != 100; ++k) {
        // Update q = F^T x
        Eigen::VectorXd q = F.transpose() * x;
        for (int i = 0; i != nU; ++i) {
            work->data->q[i] = q(i);
        }

        // Warm-start
        osqp_warm_start_x(work, U_prev.data());
        osqp_warm_start_y(work, lam_prev.data());

        // Solve
        osqp_solve(work);

        // Extract solution
        Eigen::VectorXd U_opt(nU);
        for (int i = 0; i != nU; ++i) {
            U_opt(i) = work->solution->x[i];
        }
        Eigen::VectorXd y_opt(m);
        for (int i = 0; i != m; ++i) {
            y_opt(i) = work->solution->y[i];
        }

        // Apply first control and update state with your discrete dynamics
        double u = U_opt(0);
        // x = A * x + B * u;  (using your system matrices)

        // Shift warm-start
        for (int i = 0; i != nU - nu; ++i) {
            U_prev(i) = U_opt(i + nu);
        }
        for (int i = nU - nu; i != nU; ++i) {
            U_prev(i) = U_prev(i - nu);  // repeat last value
        }
        lam_prev = y_opt;
    }

    osqp_cleanup(work);
    c_free(settings);
    c_free(data);
    return 0;
}
