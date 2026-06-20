
public class SlidingModeController {

    // Lambda and K are n x n (stored row-major)
    private final double[][] Lambda;
    private final double[][] K;
    private final double[] phi;

    public SlidingModeController(double[][] Lambda, double[][] K, double[] phi) {
        this.Lambda = Lambda;
        this.K = K;
        this.phi = phi;
    }

    // Utility: matrix-vector product y = A x
    private static double[] matVec(double[][] A, double[] x) {
        int n = x.length;
        double[] y = new double[n];
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (int j = 0; j < n; ++j) {
                sum += A[i][j] * x[j];
            }
            y[i] = sum;
        }
        return y;
    }

    // Utility: elementwise tanh
    private static double[] tanhVec(double[] x) {
        int n = x.length;
        double[] y = new double[n];
        for (int i = 0; i < n; ++i) {
            y[i] = Math.tanh(x[i]);
        }
        return y;
    }

    public double[] computeTorque(
        double[] q, double[] dq,
        double[] qd, double[] dqd, double[] ddqd
    ) {
        int n = q.length;

        // Errors
        double[] e = new double[n];
        double[] eDot = new double[n];
        for (int i = 0; i < n; ++i) {
            e[i] = q[i] - qd[i];
            eDot[i] = dq[i] - dqd[i];
        }

        // s = eDot + Lambda e
        double[] LambdaE = matVec(Lambda, e);
        double[] s = new double[n];
        for (int i = 0; i < n; ++i) {
            s[i] = eDot[i] + LambdaE[i];
        }

        // Robot dynamics (user implementation)
        double[][] M = M_robot(q);
        double[][] C = C_robot(q, dq);
        double[] g = g_robot(q);

        // tau_eq = M (ddqd - Lambda eDot) + C dq + g
        double[] LambdaEDot = matVec(Lambda, eDot);
        double[] inner = new double[n];
        for (int i = 0; i < n; ++i) {
            inner[i] = ddqd[i] - LambdaEDot[i];
        }
        double[] Minner = matVec(M, inner);
        double[] Cdq = matVec(C, dq);
        double[] tauEq = new double[n];
        for (int i = 0; i < n; ++i) {
            tauEq[i] = Minner[i] + Cdq[i] + g[i];
        }

        // tau_sw = -K sat(s/phi) with tanh
        double[] satArg = new double[n];
        for (int i = 0; i < n; ++i) {
            double phiSafe = Math.max(phi[i], 1e-6);
            satArg[i] = s[i] / phiSafe;
        }
        double[] satVal = tanhVec(satArg); // approximate sat
        double[] tauSw = new double[n];
        for (int i = 0; i < n; ++i) {
            // K is diagonal
            tauSw[i] = -K[i][i] * satVal[i];
        }

        double[] tau = new double[n];
        for (int i = 0; i < n; ++i) {
            tau[i] = tauEq[i] + tauSw[i];
        }
        return tau;
    }
}
