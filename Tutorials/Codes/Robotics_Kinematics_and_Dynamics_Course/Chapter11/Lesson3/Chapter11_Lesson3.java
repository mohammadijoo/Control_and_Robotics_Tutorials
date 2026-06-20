public class EnergyChecks {

    // User-supplied functions (placeholders)
    // M(q): returns n x n matrix
    public static double[][] M(double[] q) {
        throw new UnsupportedOperationException("Implement M(q)");
    }

    // C(q,dq): returns n x n matrix
    public static double[][] C(double[] q, double[] dq) {
        throw new UnsupportedOperationException("Implement C(q,dq)");
    }

    // g(q): returns n-vector
    public static double[] g(double[] q) {
        throw new UnsupportedOperationException("Implement g(q)");
    }

    public static double potentialEnergy(double[] q) {
        throw new UnsupportedOperationException("Implement U(q)");
    }

    // Basic linear algebra helpers --------------------------------------

    public static double[] matVec(double[][] A, double[] x) {
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

    public static double dot(double[] a, double[] b) {
        double s = 0.0;
        for (int i = 0; i < a.length; ++i) {
            s += a[i] * b[i];
        }
        return s;
    }

    public static double kineticEnergy(double[] q, double[] dq) {
        double[][] Mq = M(q);
        double[] tmp = matVec(Mq, dq);
        return 0.5 * dot(dq, tmp);
    }

    public static double totalEnergy(double[] q, double[] dq) {
        return kineticEnergy(q, dq) + potentialEnergy(q);
    }

    public static double[][] directionalMDot(double[] q, double[] dq, double eps) {
        int n = q.length;
        double[] qPlus = new double[n];
        double[] qMinus = new double[n];
        for (int i = 0; i < n; ++i) {
            qPlus[i] = q[i] + eps * dq[i];
            qMinus[i] = q[i] - eps * dq[i];
        }
        double[][] Mplus = M(qPlus);
        double[][] Mminus = M(qMinus);
        double[][] Mdot = new double[n][n];
        double inv = 1.0 / (2.0 * eps);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                Mdot[i][j] = (Mplus[i][j] - Mminus[i][j]) * inv;
            }
        }
        return Mdot;
    }

    public static double scalarSigma(double[] q, double[] dq) {
        int n = q.length;
        double[][] Mdot = directionalMDot(q, dq, 1e-6);
        double[][] Cmat = C(q, dq);
        double[][] middle = new double[n][n];
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                middle[i][j] = 0.5 * Mdot[i][j] - Cmat[i][j];
            }
        }
        double[] tmp = matVec(middle, dq);
        return dot(dq, tmp);
    }
}
      
