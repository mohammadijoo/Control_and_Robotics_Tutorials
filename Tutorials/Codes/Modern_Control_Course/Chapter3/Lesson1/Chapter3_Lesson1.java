import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SingularOps_DDRM;
import org.ejml.dense.row.decomposition.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

public class RankNullspaceDemo {
  public static void main(String[] args) {
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {1,2,3,1},
      {2,4,6,2},
      {1,1,1,0}
    });

    SingularValueDecomposition_F64<DMatrixRMaj> svd =
        DecompositionFactory_DDRM.svd(A.numRows, A.numCols, true, true, false);

    if (!svd.decompose(A.copy())) {
      throw new RuntimeException("SVD failed");
    }

    double tol = 1e-10;
    int rank = SingularOps_DDRM.rank(svd, tol);
    int n = A.numCols;
    int nullity = n - rank;

    System.out.println("rank(A) = " + rank);
    System.out.println("nullity(A) = " + nullity);
    System.out.println("rank + nullity = " + (rank + nullity) + " (should be " + n + ")");

    // Extract V and take last columns as null basis
    DMatrixRMaj V = svd.getV(null);
    DMatrixRMaj N = CommonOps_DDRM.extract(V, 0, V.numRows, rank, V.numCols);

    // Verify A*N ≈ 0
    DMatrixRMaj R = new DMatrixRMaj(A.numRows, N.numCols);
    CommonOps_DDRM.mult(A, N, R);

    double fro = CommonOps_DDRM.normF(R);
    System.out.println("||A*N||_F = " + fro);
  }
}
