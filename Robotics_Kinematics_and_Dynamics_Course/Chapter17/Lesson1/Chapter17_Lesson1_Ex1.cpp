#include <rbdl/rbdl.h>
// ...
RigidBodyDynamics::Model model;
RigidBodyDynamics::Math::VectorNd q = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
RigidBodyDynamics::Math::MatrixNd H = RigidBodyDynamics::Math::MatrixNd::Zero(model.dof_count, model.dof_count);

// Build model with a floating base joint (6D) and n joints
// ...

RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, H, /*update_kinematics=*/true);
RigidBodyDynamics::Math::VectorNd v = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
// Fill v with base twist and joint velocities
RigidBodyDynamics::Math::VectorNd p = H * v;
      
