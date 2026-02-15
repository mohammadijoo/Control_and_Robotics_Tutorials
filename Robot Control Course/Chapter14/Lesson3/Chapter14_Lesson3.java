
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ContactConsistency {

    public static DMatrixRMaj dynConsistentPinv(DMatrixRMaj M, DMatrixRMaj Jc) {
        int n = M.getNumRows();
        int nc = Jc.getNumRows();

        DMatrixRMaj Minv = new DMatrixRMaj(n, n);
        CommonOps_DDRM.invert(M, Minv);

        DMatrixRMaj JcT = new DMatrixRMaj(n, nc);
        CommonOps_DDRM.transpose(Jc, JcT);

        DMatrixRMaj temp = new DMatrixRMaj(nc, nc);
        // temp = Jc * Minv * Jc^T
        DMatrixRMaj JcMinv = new DMatrixRMaj(nc, n);
        CommonOps_DDRM.mult(Jc, Minv, JcMinv);
        CommonOps_DDRM.mult(JcMinv, JcT, temp);

        DMatrixRMaj Lambda_c = new DMatrixRMaj(nc, nc);
        CommonOps_DDRM.invert(temp, Lambda_c);

        DMatrixRMaj Jc_dyn_pinv = new DMatrixRMaj(n, nc);
        // Jc_dyn_pinv = Minv * Jc^T * Lambda_c
        DMatrixRMaj temp2 = new DMatrixRMaj(n, nc);
        CommonOps_DDRM.mult(Minv, JcT, temp2);
        CommonOps_DDRM.mult(temp2, Lambda_c, Jc_dyn_pinv);
        return Jc_dyn_pinv;
    }

    public static DMatrixRMaj contactProjector(DMatrixRMaj M, DMatrixRMaj Jc,
                                               DMatrixRMaj Jc_dyn_pinv) {
        int n = M.getNumRows();
        Jc_dyn_pinv.set(dynConsistentPinv(M, Jc));

        DMatrixRMaj Nc = CommonOps_DDRM.identity(n);
        DMatrixRMaj temp = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(Jc_dyn_pinv, Jc, temp);
        CommonOps_DDRM.subtractEquals(Nc, temp); // Nc = I - Jc_dyn_pinv * Jc
        return Nc;
    }

    public static DMatrixRMaj contactConsistentAcc(DMatrixRMaj M,
                                                   DMatrixRMaj Jc,
                                                   DMatrixRMaj Jc_dot_qdot,
                                                   DMatrixRMaj qdd_des) {
        int n = M.getNumRows();
        DMatrixRMaj Jc_dyn_pinv = new DMatrixRMaj(n, Jc.getNumRows());
        DMatrixRMaj Nc = contactProjector(M, Jc, Jc_dyn_pinv);

        DMatrixRMaj qdd_star = new DMatrixRMaj(n, 1);
        // qdd_star = Nc qdd_des - Jc_dyn_pinv Jc_dot_qdot
        DMatrixRMaj temp = new DMatrixRMaj(n, 1);
        CommonOps_DDRM.mult(Nc, qdd_des, qdd_star);
        CommonOps_DDRM.mult(Jc_dyn_pinv, Jc_dot_qdot, temp);
        CommonOps_DDRM.subtractEquals(qdd_star, temp);
        return qdd_star;
    }
}
