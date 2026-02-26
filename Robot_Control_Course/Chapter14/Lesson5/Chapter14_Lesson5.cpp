
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

// H, g, Aeq, beq, Aineq, bineq constructed elsewhere
struct WholeBodyQP {
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::MatrixXd Aeq;
  Eigen::VectorXd beq;
  Eigen::MatrixXd Aineq;
  Eigen::VectorXd bineq;
};

Eigen::VectorXd solveWholeBodyQP(const WholeBodyQP& qp) {
  const int nv = static_cast<int>(qp.H.rows());

  OsqpEigen::Solver solver;
  solver.settings().setVerbosity(false);
  solver.settings().setWarmStart(true);

  // OSQP uses sparse matrices
  solver.data()->setNumberOfVariables(nv);
  solver.data()->setNumberOfConstraints(
      qp.Aeq.rows() + qp.Aineq.rows()
  );

  // Build constraint matrix and bounds:
  Eigen::MatrixXd Aall(qp.Aeq.rows() + qp.Aineq.rows(), nv);
  Aall << qp.Aeq,
          qp.Aineq;

  Eigen::VectorXd lower(qp.Aeq.rows() + qp.Aineq.rows());
  Eigen::VectorXd upper(qp.Aeq.rows() + qp.Aineq.rows());

  // Equalities: Aeq * z = beq
  lower.head(qp.Aeq.rows()) = qp.beq;
  upper.head(qp.Aeq.rows()) = qp.beq;

  // Inequalities: Aineq * z <= bineq
  lower.tail(qp.Aineq.rows()) =
      Eigen::VectorXd::Constant(qp.Aineq.rows(), -OsqpEigen::INFTY);
  upper.tail(qp.Aineq.rows()) = qp.bineq;

  solver.data()->setHessianMatrix(qp.H.sparseView());
  solver.data()->setGradient(qp.g);
  solver.data()->setLinearConstraintsMatrix(Aall.sparseView());
  solver.data()->setLowerBound(lower);
  solver.data()->setUpperBound(upper);

  if (!solver.initSolver()) {
    throw std::runtime_error("OSQP init failed");
  }

  if (!solver.solve()) {
    throw std::runtime_error("OSQP solve failed");
  }

  return solver.getSolution();
}
