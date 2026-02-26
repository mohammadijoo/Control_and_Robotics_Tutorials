
#include <rbdl/rbdl.h>
#include <vector>

using namespace RigidBodyDynamics;
using Eigen::VectorXd;

struct PDGravityController {
    Model model;
    VectorXd Kp;
    VectorXd Kd;

    PDGravityController(const Model& model_in,
                        const VectorXd& Kp_in,
                        const VectorXd& Kd_in)
        : model(model_in), Kp(Kp_in), Kd(Kd_in) {}

    VectorXd gravityTorque(const VectorXd& q) {
        VectorXd dq  = VectorXd::Zero(model.dof_count);
        VectorXd ddq = VectorXd::Zero(model.dof_count);
        VectorXd tau = VectorXd::Zero(model.dof_count);
        InverseDynamics(model, q, dq, ddq, tau);
        return tau; // tau = g(q)
    }

    VectorXd computeTorque(const VectorXd& q,
                           const VectorXd& dq,
                           const VectorXd& q_d,
                           const VectorXd& dq_d) {
        VectorXd e  = q_d - q;
        VectorXd de = dq_d - dq;
        VectorXd tau_pd = Kp.cwiseProduct(e) + Kd.cwiseProduct(de);
        VectorXd tau_g  = gravityTorque(q);
        return tau_pd + tau_g;
    }
};

// Example usage:
// 1. Load model from URDF into RBDL::Model model;
// 2. Set Kp, Kd; create controller;
// 3. In the real-time loop, call controller.computeTorque(q, dq, q_d, dq_d).
