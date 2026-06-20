import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM;

public class InertiaConsistency {

    public static boolean isSPD(DMatrixRMaj M, double tol) {
        if (M.numRows != M.numCols) return false;
        CholeskyDecompositionInner_DDRM chol = new CholeskyDecompositionInner_DDRM(true);
        if (!chol.decompose(M)) return false;
        DMatrixRMaj L = chol.getT(null);
        for (int i = 0; i < L.numRows; ++i) {
            double diag = L.get(i, i);
            if (diag <= tol) return false;
        }
        return true;
    }

    public static DMatrixRMaj skew(DMatrixRMaj v) {
        DMatrixRMaj S = new DMatrixRMaj(3, 3);
        double x = v.get(0, 0);
        double y = v.get(1, 0);
        double z = v.get(2, 0);
        S.set(0, 0, 0.0);  S.set(0, 1, -z);  S.set(0, 2,  y);
        S.set(1, 0,  z);  S.set(1, 1,  0.0); S.set(1, 2, -x);
        S.set(2, 0, -y);  S.set(2, 1,  x);   S.set(2, 2,  0.0);
        return S;
    }

    public static DMatrixRMaj spatialInertia(double m,
                                             DMatrixRMaj c,
                                             DMatrixRMaj I_C) {
        DMatrixRMaj S = skew(c);
        DMatrixRMaj SS_T = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.mult(S, S, SS_T);

        DMatrixRMaj upperLeft = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.add(I_C, m, SS_T, upperLeft);

        DMatrixRMaj upperRight = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.scale(m, S, upperRight);

        DMatrixRMaj lowerLeft = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.transpose(upperRight, lowerLeft);

        DMatrixRMaj lowerRight = CommonOps_DDRM.identity(3);
        CommonOps_DDRM.scale(m, lowerRight, lowerRight);

        DMatrixRMaj I = new DMatrixRMaj(6, 6);
        // assemble 6x6 block matrix
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                I.set(i, j, upperLeft.get(i, j));
                I.set(i, j + 3, upperRight.get(i, j));
                I.set(i + 3, j, lowerLeft.get(i, j));
                I.set(i + 3, j + 3, lowerRight.get(i, j));
            }
        }
        return I;
    }

    public static void main(String[] args) {
        double m = 2.5;
        DMatrixRMaj c = new DMatrixRMaj(3, 1, true, new double[]{0.0, 0.05, 0.0});
        DMatrixRMaj I_C = new DMatrixRMaj(3, 3, true,
                new double[]{
                        0.01, 0.0,  0.0,
                        0.0,  0.015, 0.0,
                        0.0,  0.0,   0.008
                });

        if (m <= 0.0) {
            System.err.println("Non-positive mass");
        }

        // Symmetrize I_C
        DMatrixRMaj I_C_T = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.transpose(I_C, I_C_T);
        DMatrixRMaj I_C_sym = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.add(0.5, I_C, 0.5, I_C_T, I_C_sym);

        if (!isSPD(I_C_sym, 1e-9)) {
            System.err.println("I_C not SPD");
        }

        DMatrixRMaj I_spatial = spatialInertia(m, c, I_C_sym);
        if (!isSPD(I_spatial, 1e-9)) {
            System.err.println("Spatial inertia not SPD");
        } else {
            System.out.println("Spatial inertia SPD");
        }
    }
}
      
