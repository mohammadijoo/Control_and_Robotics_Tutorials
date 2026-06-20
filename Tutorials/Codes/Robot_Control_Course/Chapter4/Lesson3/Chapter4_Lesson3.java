
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;

public class RedundancyController {

    public static DMatrixRMaj pseudoinverse(DMatrixRMaj J, double damping) {
        int m = J.numRows;
        int n = J.numCols;
        DMatrixRMaj JJt = new DMatrixRMaj(m, m);
        CommonOps_DDRM.mult(J, J, JJt, true, false); // JJt = J * J^T

        DMatrixRMaj A = JJt.copy();
        if (damping > 0.0) {
            for (int i = 0; i < m; ++i) {
                A.set(i, i, A.get(i, i) + damping * damping);
            }
        }

        // Invert A via Cholesky (since JJt is SPD if full rank and damped)
        CholeskyDecompositionInner_DDRM chol = new CholeskyDecompositionInner_DDRM(true);
        if (!chol.decompose(A)) {
            throw new RuntimeException("Cholesky failed (Jacobian not full row rank?)");
        }
        DMatrixRMaj Ainv = new DMatrixRMaj(m, m);
        chol.invert(Ainv);

        DMatrixRMaj Jt = new DMatrixRMaj(n, m);
        CommonOps_DDRM.transpose(J, Jt);

        DMatrixRMaj Jsharp = new DMatrixRMaj(n, m);
        CommonOps_DDRM.mult(Jt, Ainv, Jsharp);
        return Jsharp;
    }

    public static DMatrixRMaj nullProjector(DMatrixRMaj J, double damping) {
        DMatrixRMaj Jsharp = pseudoinverse(J, damping);
        int n = J.numCols;
        DMatrixRMaj I = CommonOps_DDRM.identity(n);
        DMatrixRMaj N = I.copy();

        DMatrixRMaj JJ = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(Jsharp, J, JJ);
        CommonOps_DDRM.subtractEquals(N, JJ); // N = I - Jsharp * J
        return N;
    }
}
