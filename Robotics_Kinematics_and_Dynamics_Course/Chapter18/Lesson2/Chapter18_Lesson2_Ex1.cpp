#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

// ...

KDL::Chain chain; // filled from robot description
// ...
KDL::ChainJntToJacSolver jac_solver(chain);
KDL::JntArray q(chain.getNrOfJoints());
// set q(i) from current joint configuration
KDL::Jacobian J(chain.getNrOfJoints());
jac_solver.JntToJac(q, J); // 6 x n geometric Jacobian at the end-effector
      
