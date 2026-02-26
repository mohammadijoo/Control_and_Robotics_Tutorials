
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecomposition_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;

public class FiniteHorizonLQR {

    public static class Result {
        public DMatrixRMaj[] P;
        public DMatrixRMaj[] K;
    }

    public static Result lqr(DMatrixRMaj Ad,
                             DMatrixRMaj Bd,
                             DMatrixRMaj Q,
                             DMatrixRMaj R,
                             int N) {

        int n = Ad.numRows;
        int m = Bd.numCols;

        Result res = new Result();
        res.P = new DMatrixRMaj[N + 1];
        res.K = new DMatrixRMaj[N];

        res.P[N] = Q.copy();

        DMatrixRMaj Bt = new DMatrixRMaj(m, n);
        DMatrixRMaj At = new DMatrixRMaj(n, n);
        CommonOps_DDRM.transpose(Bd, Bt);
        CommonOps_DDRM.transpose(Ad, At);

        for (int k = N - 1; k >= 0; --k) {
            DMatrixRMaj Pnext = res.P[k + 1];
            DMatrixRMaj BtPnext = new DMatrixRMaj(m, n);
            CommonOps_DDRM.mult(Bt, Pnext, BtPnext);

            DMatrixRMaj S = new DMatrixRMaj(m, m);
            DMatrixRMaj BtPnextB = new DMatrixRMaj(m, m);
            CommonOps_DDRM.mult(BtPnext, Bd, BtPnextB);
            CommonOps_DDRM.add(R, BtPnextB, S);

            // Solve S * Kk = BtPnext * Ad
            DMatrixRMaj BtPnextAd = new DMatrixRMaj(m, n);
            CommonOps_DDRM.mult(BtPnext, Ad, BtPnextAd);

            DMatrixRMaj Kk = BtPnextAd.copy();
            CholeskyDecomposition_DDRM chol = new CholeskyDecompositionInner_DDRM();
            chol.decompose(S);
            chol.solve(Kk); // Kk = S^{-1} * BtPnextAd

            res.K[k] = Kk;

            // P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B Kk
            DMatrixRMaj APnext = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(Pnext, Ad, APnext);
            DMatrixRMaj AtPnextAd = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(At, APnext, AtPnextAd);

            DMatrixRMaj BK = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(Bd, Kk, BK);
            DMatrixRMaj PnextBK = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(Pnext, BK, PnextBK);
            DMatrixRMaj AtPnextBK = new DMatrixRMaj(n, n);
            CommonOps_DDRM.mult(At, PnextBK, AtPnextBK);

            DMatrixRMaj Pk = Q.copy();
            CommonOps_DDRM.addEquals(Pk, AtPnextAd);
            CommonOps_DDRM.subtractEquals(Pk, AtPnextBK);
            res.P[k] = Pk;
        }

        return res;
    }

    public static void main(String[] args) {
        double I = 0.5;
        double b = 0.1;
        double dt = 0.002;
        double T = 2.0;
        int N = (int)(T / dt);

        DMatrixRMaj A = new DMatrixRMaj(2, 2, true,
                0.0, 1.0,
                0.0, -b / I);
        DMatrixRMaj B = new DMatrixRMaj(2, 1, true,
                0.0,
                1.0 / I);

        DMatrixRMaj Ad = A.copy();
        CommonOps_DDRM.scale(dt, Ad);
        CommonOps_DDRM.addEquals(Ad, 1.0, CommonOps_DDRM.identity(2));

        DMatrixRMaj Bd = B.copy();
        CommonOps_DDRM.scale(dt, Bd);

        DMatrixRMaj Q = new DMatrixRMaj(2, 2, true,
                100.0, 0.0,
                0.0, 10.0);
        DMatrixRMaj R = new DMatrixRMaj(1, 1, true, 1.0);

        Result res = lqr(Ad, Bd, Q, R, N);

        // Closed-loop simulation can then be carried out as in the C++ example.
    }
}
