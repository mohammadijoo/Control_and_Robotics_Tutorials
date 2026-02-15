import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SingularOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SingularValueDecomposition_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;

public class GraspQuality {

    public static DMatrixRMaj skew(double[] p) {
        DMatrixRMaj S = new DMatrixRMaj(3,3);
        double px = p[0], py = p[1], pz = p[2];
        S.set(0,0, 0.0);   S.set(0,1, -pz);  S.set(0,2,  py);
        S.set(1,0,  pz);   S.set(1,1, 0.0);  S.set(1,2, -px);
        S.set(2,0, -py);   S.set(2,1,  px);  S.set(2,2, 0.0);
        return S;
    }

    public static DMatrixRMaj graspMapBlock(double[] p_i, DMatrixRMaj R_i) {
        DMatrixRMaj G_i = new DMatrixRMaj(6,3);
        // Upper block = R_i
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                G_i.set(r, c, R_i.get(r, c));
            }
        }
        // Lower block = skew(p_i) * R_i
        DMatrixRMaj S = skew(p_i);
        DMatrixRMaj SR = new DMatrixRMaj(3,3);
        CommonOps_DDRM.mult(S, R_i, SR);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                G_i.set(r + 3, c, SR.get(r, c));
            }
        }
        return G_i;
    }

    public static DMatrixRMaj linearizedFrictionDirections(double mu, int kDirs) {
        DMatrixRMaj D = new DMatrixRMaj(3, kDirs);
        double alpha = Math.atan(mu);
        double fn = Math.cos(alpha);
        double ft = Math.sin(alpha);
        for (int j = 0; j < kDirs; ++j) {
            double theta = 2.0 * Math.PI * j / (double) kDirs;
            double tx = ft * Math.cos(theta);
            double ty = ft * Math.sin(theta);
            D.set(0, j, tx);
            D.set(1, j, ty);
            D.set(2, j, fn);
        }
        return D;
    }

    public static double isotropyMetric(DMatrixRMaj G) {
        SingularValueDecomposition_DDRM svd =
            new SvdImplicitQrDecompose_DDRM(true, true, true, false);
        if (!svd.decompose(G)) {
            return 0.0;
        }
        double[] sv = svd.getSingularValues();
        double sMin = Double.POSITIVE_INFINITY;
        double sMax = 0.0;
        for (double s : sv) {
            sMin = Math.min(sMin, s);
            sMax = Math.max(sMax, s);
        }
        if (sMax < 1e-12) return 0.0;
        return sMin / sMax;
    }

    public static void main(String[] args) {
        // For brevity, contact geometry setup omitted; assume we have a
        // 6 x N primitive wrench matrix G_approx.

        // Example placeholder: random 6 x 12 matrix
        DMatrixRMaj G = new DMatrixRMaj(6, 12);
        java.util.Random rng = new java.util.Random(0);
        for (int i = 0; i < G.getNumElements(); ++i) {
            G.set(i, rng.nextGaussian());
        }

        double qIso = isotropyMetric(G);
        System.out.println("Isotropy metric Q_iso = " + qIso);
    }
}
      
