
#include <Eigen/Dense>
#include <qpOASES.hpp>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class JointCBFQPMultiDOF {
public:
    JointCBFQPMultiDOF(const VectorXd& q_min,
                       const VectorXd& q_max,
                       double gamma,
                       const VectorXd& u_min,
                       const VectorXd& u_max)
    : q_min_(q_min), q_max_(q_max),
      gamma_(gamma),
      u_min_(u_min), u_max_(u_max),
      n_(q_min.size()),
      qp_(n_, 2*n_) // n vars, 2n inequality constraints
    {
        H_.setIdentity(n_, n_); // objective 0.5 * (u - u_nom)^T (u - u_nom)
    }

    VectorXd solveQP(const VectorXd& q,
                     const VectorXd& u_nom)
    {
        using namespace qpOASES;

        // Build QP: min 0.5 u^T H u + g^T u
        // with H = 2I, g = -2 u_nom (so objective = ||u - u_nom||^2)
        MatrixXd H = 2.0 * H_;
        VectorXd g = -2.0 * u_nom;

        // CBF-based bounds: u_i >= -gamma (q_i - q_min_i)
        //                    u_i <=  gamma (q_max_i - q_i)
        VectorXd cbf_lb(n_), cbf_ub(n_);
        cbf_lb = -gamma_ * (q - q_min_);
        cbf_ub =  gamma_ * (q_max_ - q);

        // Inequalities of form A u <= b
        // We use:
        //   u_i <= cbf_ub_i
        //  -u_i <= -cbf_lb_i  (equivalently u_i >= cbf_lb_i)
        MatrixXd A(2*n_, n_);
        VectorXd b(2*n_);
        A.setZero();
        b.setZero();
        // Upper CBF bounds
        A.topRows(n_) =  MatrixXd::Identity(n_, n_);
        b.head(n_)   =  cbf_ub;
        // Lower CBF bounds
        A.bottomRows(n_) = -MatrixXd::Identity(n_, n_);
        b.tail(n_)       = -cbf_lb;

        // Variable bounds (actuator limits)
        VectorXd lb = u_min_;
        VectorXd ub = u_max_;

        // Convert Eigen to qpOASES arrays
        real_t* H_qp = const_cast<real_t*>(H.data());
        real_t* g_qp = const_cast<real_t*>(g.data());
        real_t* A_qp = const_cast<real_t*>(A.data());
        real_t* lb_qp = const_cast<real_t*>(lb.data());
        real_t* ub_qp = const_cast<real_t*>(ub.data());
        real_t* lbA_qp = nullptr;
        real_t* ubA_qp = b.data(); // lower bounds for A u, here  -inf < A u <= b

        // For simplicity, we set lower bounds on A u to -inf
        VectorXd lbA_vec = VectorXd::Constant(2*n_, -1e9);
        lbA_qp = lbA_vec.data();

        int nWSR = 20;
        qp_.init(H_qp, g_qp, A_qp,
                 lb_qp, ub_qp,
                 lbA_qp, ubA_qp,
                 nWSR);

        VectorXd u_opt(n_);
        qp_.getPrimalSolution(u_opt.data());
        return u_opt;
    }

private:
    VectorXd q_min_, q_max_;
    double gamma_;
    VectorXd u_min_, u_max_;
    int n_;
    MatrixXd H_;
    qpOASES::QProblem qp_;
};
