import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.linsol.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

public class JordanChainDemo {
  public static void main(String[] args) {
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {2, 1, 0},
      {0, 2, 0},
      {0, 0, 3}
    });

    double lambda = 2.0;

    DMatrixRMaj I = CommonOps_DDRM.identity(3);
    DMatrixRMaj M = new DMatrixRMaj(3,3);
    CommonOps_DDRM.scale(lambda, I, M);
    CommonOps_DDRM.subtract(A, M, M); // M = A - lambda I

    // v1 = [1,0,0]^T is an eigenvector for lambda=2 in this example
    DMatrixRMaj v1 = new DMatrixRMaj(new double[][]{ {1},{0},{0} });

    // Solve M v2 = v1
    DMatrixRMaj v2 = new DMatrixRMaj(3,1);
    LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.lu(3);
    solver.setA(M);
    solver.solve(v1, v2);

    // w = [0,0,1]^T eigenvector for lambda=3
    DMatrixRMaj w = new DMatrixRMaj(new double[][]{ {0},{0},{1} });

    // P = [v1 v2 w]
    DMatrixRMaj P = new DMatrixRMaj(3,3);
    P.set(0,0, v1.get(0,0)); P.set(1,0, v1.get(1,0)); P.set(2,0, v1.get(2,0));
    P.set(0,1, v2.get(0,0)); P.set(1,1, v2.get(1,0)); P.set(2,1, v2.get(2,0));
    P.set(0,2, w.get(0,0));  P.set(1,2, w.get(1,0));  P.set(2,2, w.get(2,0));

    // J = P^{-1} A P
    DMatrixRMaj Pinv = new DMatrixRMaj(3,3);
    CommonOps_DDRM.invert(P, Pinv);

    DMatrixRMaj AP = new DMatrixRMaj(3,3);
    CommonOps_DDRM.mult(A, P, AP);

    DMatrixRMaj J = new DMatrixRMaj(3,3);
    CommonOps_DDRM.mult(Pinv, AP, J);

    System.out.println("J = ");
    J.print();
  }
}
