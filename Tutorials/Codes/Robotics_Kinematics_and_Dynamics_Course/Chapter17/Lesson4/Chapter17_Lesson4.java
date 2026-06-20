import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ContactDynamics {

    public static class ContactSystem {
        public DMatrixRMaj M;   // (n x n)
        public DMatrixRMaj h;   // (n x 1)
        public DMatrixRMaj Jc;  // (m x n)
    }

    public static void solveContactDynamics(ContactSystem sys,
                                            DMatrixRMaj tau,
                                            DMatrixRMaj fExt,
                                            DMatrixRMaj qd,
                                            DMatrixRMaj qddotOut,
                                            DMatrixRMaj lambdaOut) {
        int n = sys.M.numRows;
        int m = sys.Jc.numRows;

        DMatrixRMaj KKT = new DMatrixRMaj(n + m, n + m);
        CommonOps_DDRM.fill(KKT, 0.0);

        // Top-left block: M
        CommonOps_DDRM.insert(sys.M, KKT, 0, 0);

        // Top-right block: -Jc.T
        DMatrixRMaj JcT = new DMatrixRMaj(sys.Jc.numCols, sys.Jc.numRows);
        CommonOps_DDRM.transpose(sys.Jc, JcT);
        CommonOps_DDRM.scale(-1.0, JcT);
        CommonOps_DDRM.insert(JcT, KKT, 0, n);

        // Bottom-left block: Jc
        CommonOps_DDRM.insert(sys.Jc, KKT, n, 0);

        // RHS
        DMatrixRMaj rhs = new DMatrixRMaj(n + m, 1);
        DMatrixRMaj temp = new DMatrixRMaj(n, 1);

        // tau + fExt - h
        CommonOps_DDRM.add(tau, fExt, temp);
        CommonOps_DDRM.subtractEquals(temp, sys.h);
        CommonOps_DDRM.insert(temp, rhs, 0, 0);

        // Constraint acceleration term (assumed zero)
        for (int i = 0; i < m; ++i) {
            rhs.set(n + i, 0, 0.0);
        }

        // Solve KKT * sol = rhs
        DMatrixRMaj sol = new DMatrixRMaj(n + m, 1);
        CommonOps_DDRM.solve(KKT, rhs, sol);

        for (int i = 0; i < n; ++i) {
            qddotOut.set(i, 0, sol.get(i, 0));
        }
        for (int i = 0; i < m; ++i) {
            lambdaOut.set(i, 0, sol.get(n + i, 0));
        }
    }
}
      
