import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

public class EigenLesson {
  public static void main(String[] args) {
    // Example matrix
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {4.0, 1.0, 0.0},
      {1.0, 3.0, 1.0},
      {0.0, 1.0, 2.0}
    });

    // 1) Eigen-decomposition (general; eigenvectors may be complex in general problems)
    EigenDecomposition_F64<DMatrixRMaj> eig =
        DecompositionFactory_DDRM.eig(A.numRows, true);

    if (!eig.decompose(A)) {
      System.out.println("Eigendecomposition failed.");
      return;
    }

    System.out.println("Eigenvalues:");
    for (int i = 0; i < A.numRows; i++) {
      System.out.println("  " + eig.getEigenvalue(i));
    }

    // 2) Power iteration (from scratch) for dominant eigenpair (works well with spectral gap)
    DMatrixRMaj x = new DMatrixRMaj(A.numRows, 1);
    for (int i = 0; i < A.numRows; i++) x.set(i, 0, Math.random());
    normalize2(x);

    double lamOld = 0.0;
    int maxIter = 2000;
    double tol = 1e-10;

    DMatrixRMaj y = new DMatrixRMaj(A.numRows, 1);
    for (int k = 0; k < maxIter; k++) {
      CommonOps_DDRM.mult(A, x, y);
      normalize2(y);

      // Rayleigh quotient lambda = x^T A x (reuse y = A x if desired)
      CommonOps_DDRM.mult(A, y, x);  // x <- A*y (temporary)
      double lam = dot(y, x);        // y^T (A y)
      // restore x to current normalized eigenvector estimate:
      x.set(y);

      if (Math.abs(lam - lamOld) < tol) {
        System.out.println("Power iteration converged in " + (k+1) + " iterations.");
        System.out.println("lambda_hat = " + lam);
        System.out.println("v_hat (approx) = ");
        x.print();
        break;
      }
      lamOld = lam;
    }
  }

  static double dot(DMatrixRMaj a, DMatrixRMaj b) {
    double s = 0.0;
    for (int i = 0; i < a.getNumElements(); i++) s += a.get(i) * b.get(i);
    return s;
  }

  static void normalize2(DMatrixRMaj v) {
    double n2 = NormOps_DDRM.normF(v);
    if (n2 > 0) CommonOps_DDRM.scale(1.0 / n2, v);
  }
}
