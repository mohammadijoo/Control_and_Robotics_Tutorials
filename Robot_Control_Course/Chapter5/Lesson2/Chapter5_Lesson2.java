
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class EqualityConstrainedControl {

    public static class Result {
        public DMatrixRMaj tau;
        public DMatrixRMaj lambda;
    }

    public static Result computeProjectedTorque(
            DMatrixRMaj M,
            DMatrixRMaj h,
            DMatrixRMaj Jc,
            DMatrixRMaj bc,
            DMatrixRMaj tau0,
            double eps)
    {
        int n = M.getNumRows();
        int m = Jc.getNumRows();

        // Minv*(tau0 - h) via solve
        DMatrixRMaj tau0_minus_h = new DMatrixRMaj(n, 1);
        CommonOps_DDRM.subtract(tau0, h, tau0_minus_h);

        DMatrixRMaj Minv_tau0_minus_h = new DMatrixRMaj(n, 1);
        CommonOps_DDRM.solve(M, tau0_minus_h, Minv_tau0_minus_h);

        // Minv*Jc^T
        DMatrixRMaj JcT = new DMatrixRMaj(n, m);
        CommonOps_DDRM.transpose(Jc, JcT);

        DMatrixRMaj Minv_JcT = new DMatrixRMaj(n, m);
        CommonOps_DDRM.solve(M, JcT, Minv_JcT);

        // JMJT = Jc * Minv * Jc^T
        DMatrixRMaj JMJT = new DMatrixRMaj(m, m);
        CommonOps_DDRM.mult(Jc, Minv_JcT, JMJT);

        // Regularize and invert JMJT
        for (int i = 0; i < m; ++i) {
            double v = JMJT.get(i, i);
            JMJT.set(i, i, v + eps);
        }
        DMatrixRMaj Lambda_c = new DMatrixRMaj(m, m);
        CommonOps_DDRM.invert(JMJT, Lambda_c);

        // lambda = -Lambda_c * (Jc * Minv_tau0_minus_h + bc)
        DMatrixRMaj tmp = new DMatrixRMaj(m, 1);
        CommonOps_DDRM.mult(Jc, Minv_tau0_minus_h, tmp);
        CommonOps_DDRM.addEquals(tmp, 1.0, bc);

        DMatrixRMaj lambda = new DMatrixRMaj(m, 1);
        CommonOps_DDRM.mult(Lambda_c, tmp, lambda);
        CommonOps_DDRM.scale(-1.0, lambda);

        // tau = tau0 + Jc^T * lambda
        DMatrixRMaj tau = tau0.copy();
        DMatrixRMaj JcT_lambda = new DMatrixRMaj(n, 1);
        CommonOps_DDRM.mult(JcT, lambda, JcT_lambda);
        CommonOps_DDRM.addEquals(tau, 1.0, JcT_lambda);

        Result res = new Result();
        res.tau = tau;
        res.lambda = lambda;
        return res;
    }
}
