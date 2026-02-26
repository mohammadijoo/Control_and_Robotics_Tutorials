import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SingularOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;

public class RobotPCA {

    public static class PCABasis {
        public DMatrixRMaj qMean; // (1 x n)
        public DMatrixRMaj V_r;   // (n x r)
    }

    public static PCABasis computePCA(DMatrixRMaj Q, double energyThreshold) {
        int K = Q.numRows;
        int n = Q.numCols;

        // Compute column-wise mean
        DMatrixRMaj qMean = new DMatrixRMaj(1, n);
        for (int j = 0; j < n; ++j) {
            double sum = 0.0;
            for (int i = 0; i < K; ++i) {
                sum += Q.get(i, j);
            }
            qMean.set(0, j, sum / K);
        }

        // Center data: Qc = Q - 1 * qMean
        DMatrixRMaj Qc = Q.copy();
        for (int i = 0; i < K; ++i) {
            for (int j = 0; j < n; ++j) {
                Qc.set(i, j, Qc.get(i, j) - qMean.get(0, j));
            }
        }

        // SVD: Qc = U * S * V^T
        SvdImplicitQrDecompose_DDRM svd =
                new SvdImplicitQrDecompose_DDRM(true, true, false, false);
        svd.decompose(Qc);

        DMatrixRMaj U = svd.getU(null, false);
        DMatrixRMaj W = svd.getW(null);
        DMatrixRMaj Vt = svd.getV(null, true);

        int m = W.numRows;
        double total = 0.0;
        double[] s2 = new double[m];
        for (int i = 0; i < m; ++i) {
            double s = W.get(i, i);
            s2[i] = s * s;
            total += s2[i];
        }

        double cum = 0.0;
        int r = 0;
        for (int i = 0; i < m; ++i) {
            cum += s2[i];
            if (cum / total >= energyThreshold) {
                r = i + 1;
                break;
            }
        }
        if (r == 0) r = m;

        // V_r is first r columns of V (V is transpose of Vt)
        DMatrixRMaj V = new DMatrixRMaj(n, n);
        CommonOps_DDRM.transpose(Vt, V);

        DMatrixRMaj V_r = new DMatrixRMaj(n, r);
        CommonOps_DDRM.extract(V, 0, n, 0, r, V_r, 0, 0);

        PCABasis basis = new PCABasis();
        basis.qMean = qMean;
        basis.V_r = V_r;
        return basis;
    }
}
      
