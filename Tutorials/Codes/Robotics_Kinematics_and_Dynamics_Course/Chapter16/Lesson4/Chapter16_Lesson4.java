import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ClosedChainDynamics {

    // User-defined: return M(q), h(q,qdot) = C(q,qdot) qdot + g(q)
    static void computeDynamics(double[] q, double[] qdot,
                                DMatrixRMaj M, DMatrixRMaj h) {
        // Fill M and h appropriately
    }

    // User-defined: Jc(q) and Jc_dot(q,qdot) qdot
    static void computeConstraints(double[] q, double[] qdot,
                                   DMatrixRMaj Jc, DMatrixRMaj JcDotQdot) {
        // Fill Jc and JcDotQdot
    }

    static DMatrixRMaj Sa() {
        // Build selection matrix for actuated joints
        return null;
    }

    static DMatrixRMaj tauA(double t, double[] q, double[] qdot) {
        // Return actuator torques as column vector
        return null;
    }

    public static double[] rhs(double t, double[] x) {
        int n = x.length / 2;
        double[] q = new double[n];
        double[] qdot = new double[n];
        System.arraycopy(x, 0,     q,    0, n);
        System.arraycopy(x, n,     qdot, 0, n);

        // Build matrices
        DMatrixRMaj M = new DMatrixRMaj(n, n);
        DMatrixRMaj h = new DMatrixRMaj(n, 1);
        computeDynamics(q, qdot, M, h);

        DMatrixRMaj Jc = new DMatrixRMaj(0, 0);       // will be resized
        DMatrixRMaj JcDotQdot = new DMatrixRMaj(0, 1);
        computeConstraints(q, qdot, Jc, JcDotQdot);

        int m = Jc.getNumRows();
        DMatrixRMaj S = Sa();
        DMatrixRMaj tau = tauA(t, q, qdot);

        // Build saddle-point matrix K and rhs b
        DMatrixRMaj K = new DMatrixRMaj(n + m, n + m);
        CommonOps_DDRM.fill(K, 0.0);
        CommonOps_DDRM.insert(M, K, 0, 0);

        DMatrixRMaj JcT = new DMatrixRMaj(n, m);
        CommonOps_DDRM.transpose(Jc, JcT);
        CommonOps_DDRM.insert(JcT, K, 0, n);
        CommonOps_DDRM.insert(Jc,  K, n, 0);

        DMatrixRMaj b = new DMatrixRMaj(n + m, 1);
        DMatrixRMaj tmp = new DMatrixRMaj(n, 1);

        // b_head = S^T tau - h
        DMatrixRMaj ST = new DMatrixRMaj(n, S.getNumRows());
        CommonOps_DDRM.transpose(S, ST);
        CommonOps_DDRM.mult(ST, tau, tmp);
        CommonOps_DDRM.subtract(tmp, h, tmp);
        CommonOps_DDRM.insert(tmp, b, 0, 0);

        // b_tail = -Jc_dot(q,qdot) qdot
        DMatrixRMaj minusJcDotQdot = new DMatrixRMaj(m, 1);
        CommonOps_DDRM.changeSign(JcDotQdot, minusJcDotQdot);
        CommonOps_DDRM.insert(minusJcDotQdot, b, n, 0);

        // Solve K [qddot; lambda] = b
        DMatrixRMaj sol = b.copy();
        CommonOps_DDRM.solve(K, b, sol);

        double[] xdot = new double[2 * n];
        // xdot_head = qdot
        System.arraycopy(qdot, 0, xdot, 0, n);
        // xdot_tail = qddot
        for (int i = 0; i < n; ++i) {
            xdot[n + i] = sol.get(i, 0);
        }
        return xdot;
    }
}
      
