#include <rbdl/rbdl.h>
#include <vector>

struct ResidualModel {
  virtual ~ResidualModel() {}
  virtual void compute(const RigidBodyDynamics::Math::VectorNd& q,
                       const RigidBodyDynamics::Math::VectorNd& dq,
                       const RigidBodyDynamics::Math::VectorNd& ddq,
                       RigidBodyDynamics::Math::VectorNd& r_out) const = 0;
};

struct ZeroResidual : public ResidualModel {
  void compute(const RigidBodyDynamics::Math::VectorNd& q,
               const RigidBodyDynamics::Math::VectorNd& dq,
               const RigidBodyDynamics::Math::VectorNd& ddq,
               RigidBodyDynamics::Math::VectorNd& r_out) const override {
    r_out.setZero();
  }
};

struct ComplexRobotModelCpp {
  RigidBodyDynamics::Model model;
  ResidualModel* residual;

  ComplexRobotModelCpp(const RigidBodyDynamics::Model& m,
                       ResidualModel* residual_model)
    : model(m), residual(residual_model) {}

  void inverse_dynamics(const RigidBodyDynamics::Math::VectorNd& q,
                        const RigidBodyDynamics::Math::VectorNd& dq,
                        const RigidBodyDynamics::Math::VectorNd& ddq,
                        RigidBodyDynamics::Math::VectorNd& tau_out) {
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    VectorNd tau_nom = VectorNd::Zero(model.qdot_size);
    RigidBodyDynamics::InverseDynamics(model, q, dq, ddq, tau_nom);

    VectorNd r = VectorNd::Zero(model.qdot_size);
    if (residual) {
      residual->compute(q, dq, ddq, r);
    }
    tau_out = tau_nom - r;
  }
};
      
