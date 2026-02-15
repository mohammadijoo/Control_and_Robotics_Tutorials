public final class JerkUtils {

    public static double[][] finiteDifferenceJerk(double[][] qSamples, double dt) {
        int Nplus1 = qSamples.length;
        if (Nplus1 < 4) {
            throw new IllegalArgumentException("Need at least 4 samples");
        }
        int nJoints = qSamples[0].length;
        int N = Nplus1 - 1;
        double[][] jerk = new double[N - 2][nJoints];

        double dt3 = dt * dt * dt;

        for (int k = 1; k < N - 1; ++k) {
            for (int j = 0; j < nJoints; ++j) {
                double qkp2 = qSamples[k + 2][j];
                double qkp1 = qSamples[k + 1][j];
                double qk   = qSamples[k][j];
                double qkm1 = qSamples[k - 1][j];
                jerk[k - 1][j] = (qkp2 - 3.0 * qkp1 + 3.0 * qk - qkm1) / dt3;
            }
        }
        return jerk;
    }

    public static double jerkSmoothnessIndex(double[][] qSamples, double dt, double[][] W) {
        double[][] jerk = finiteDifferenceJerk(qSamples, dt);
        int Tsteps = jerk.length;
        int nJoints = jerk[0].length;

        double[][] Wlocal;
        if (W == null) {
            Wlocal = new double[nJoints][nJoints];
            for (int i = 0; i < nJoints; ++i) {
                Wlocal[i][i] = 1.0;
            }
        } else {
            Wlocal = W;
        }

        double J = 0.0;
        for (int k = 0; k < Tsteps; ++k) {
            for (int i = 0; i < nJoints; ++i) {
                for (int j = 0; j < nJoints; ++j) {
                    J += jerk[k][i] * Wlocal[i][j] * jerk[k][j];
                }
            }
        }
        J *= dt;
        return J;
    }

    public static void main(String[] args) {
        double T = 2.0;
        double dt = 0.01;
        int Nplus1 = (int) Math.round(T / dt) + 1;
        double[][] qSamples = new double[Nplus1][1];

        for (int k = 0; k < Nplus1; ++k) {
            double t = k * dt;
            double s = t / T;
            double q = 10.0 * Math.pow(s, 3)
                     - 15.0 * Math.pow(s, 4)
                     + 6.0 * Math.pow(s, 5);
            qSamples[k][0] = q;
        }

        double J = jerkSmoothnessIndex(qSamples, dt, null);
        System.out.println("Jerk-based smoothness index J = " + J);
    }
}
      
