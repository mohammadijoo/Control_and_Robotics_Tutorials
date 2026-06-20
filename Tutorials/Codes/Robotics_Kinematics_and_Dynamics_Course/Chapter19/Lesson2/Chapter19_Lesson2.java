import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ExcitationDesign {

    static double[][] multiSineJoint(double[] t, double q0,
                                     double[] A, double[] W, double[] PHI0) {
        int N = t.length;
        int M = W.length;
        double[] q = new double[N];
        double[] qd = new double[N];
        double[] qdd = new double[N];
        for (int k = 0; k < N; ++k) {
            q[k] = q0;
            qd[k] = 0.0;
            qdd[k] = 0.0;
        }
        for (int m = 0; m < M; ++m) {
            double w = W[m];
            double a = A[m];
            double ph = PHI0[m];
            for (int k = 0; k < N; ++k) {
                double arg = w * t[k] + ph;
                double s = Math.sin(arg);
                double c = Math.cos(arg);
                q[k]  += a * s;
                qd[k] += a * w * c;
                qdd[k] += -a * w * w * s;
            }
        }
        return new double[][]{q, qd, qdd};
    }

    // Placeholder for symbolic regressor of a 2R arm: Y(q, qd, qdd) in R^(2 x p)
    static DMatrixRMaj computeRegressor(double q1, double q2,
                                        double qd1, double qd2,
                                        double qdd1, double qdd2) {
        int p = 6; // example minimal parameter dimension
        DMatrixRMaj Y = new DMatrixRMaj(2, p);
        // Fill Y with appropriate basis functions of q, qd, qdd
        // (e.g., cos(q2), 2*qd1*qd2, qdd1, etc.)
        // ...
        return Y;
    }

    public static void main(String[] args) {
        double T = 5.0;
        double dt = 0.002;
        int N = (int) (T / dt);
        double[] t = new double[N];
        for (int k = 0; k < N; ++k) t[k] = k * dt;

        int M = 3;
        double[] A1 = {0.4, 0.3, 0.2};
        double[] W = {1.0, 2.5, 4.0};
        double[] PHI0 = {0.0, Math.PI / 3.0, Math.PI / 2.0};

        double[][] joint1 = multiSineJoint(t, 0.0, A1, W, PHI0);
        double[][] joint2 = multiSineJoint(t, 0.0, A1, W, PHI0); // copy for simplicity

        int p = 6;
        DMatrixRMaj Phi = new DMatrixRMaj(2 * N, p);
        DMatrixRMaj Yk;
        for (int k = 0; k < N; ++k) {
            double q1 = joint1[0][k], qd1 = joint1[1][k], qdd1 = joint1[2][k];
            double q2 = joint2[0][k], qd2 = joint2[1][k], qdd2 = joint2[2][k];
            Yk = computeRegressor(q1, q2, qd1, qd2, qdd1, qdd2);
            // copy into Phi at rows 2k and 2k+1
            for (int r = 0; r < 2; ++r) {
                for (int c = 0; c < p; ++c) {
                    Phi.set(2 * k + r, c, Yk.get(r, c));
                }
            }
        }

        // F = Phi^T Phi
        DMatrixRMaj F = new DMatrixRMaj(p, p);
        CommonOps_DDRM.multTransA(Phi, Phi, F);

        // eigenvalues (EJML has symmetric eigensolvers)
        org.ejml.dense.row.decomposition.eig.SymmetricQRAlgorithmDecomposition_DDRM eig =
                new org.ejml.dense.row.decomposition.eig.SymmetricQRAlgorithmDecomposition_DDRM(p, true);
        eig.decompose(F);
        double lambdaMin = Double.POSITIVE_INFINITY;
        double lambdaMax = 0.0;
        for (int i = 0; i < p; ++i) {
            double val = eig.getEigenvalue(i).getReal();
            lambdaMin = Math.min(lambdaMin, val);
            lambdaMax = Math.max(lambdaMax, val);
        }
        double cond = lambdaMax / lambdaMin;
        System.out.println("lambda_min = " + lambdaMin);
        System.out.println("cond(F)    = " + cond);
    }
}
      
