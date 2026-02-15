import java.util.Arrays;

public class SensorMath {

    // Encoder counts -> angle (rad)
    public static double[] countsToAngle(int[] counts, int N) {
        double scale = 2.0 * Math.PI / (double) N;
        double[] theta = new double[counts.length];
        for (int k = 0; k < counts.length; k++) {
            theta[k] = scale * counts[k];
        }
        return theta;
    }

    // Gyro integration (1D)
    public static double[] integrateGyro(double[] omegaMeas, double Ts, double theta0) {
        double[] theta = new double[omegaMeas.length];
        theta[0] = theta0;
        for (int k = 1; k < omegaMeas.length; k++) {
            theta[k] = theta[k-1] + Ts * omegaMeas[k-1];
        }
        return theta;
    }

    // Wrench reconstruction using normal equations
    // (for small systems; in practice use a linear algebra library)
    public static double[] wrenchLS(double[][] S, double[] v) {
        int m = S.length;
        int p = S[0].length; // p=6
        double[][] A = new double[p][p];
        double[] b = new double[p];

        for (int i = 0; i < m; i++) {
            for (int r = 0; r < p; r++) {
                b[r] += S[i][r] * v[i];
                for (int c = 0; c < p; c++) {
                    A[r][c] += S[i][r] * S[i][c];
                }
            }
        }

        // Solve A w = b by naive Gaussian elimination
        double[] w = Arrays.copyOf(b, p);
        for (int k = 0; k < p; k++) {
            double pivot = A[k][k];
            for (int j = k; j < p; j++) A[k][j] /= pivot;
            w[k] /= pivot;
            for (int i = k+1; i < p; i++) {
                double f = A[i][k];
                for (int j = k; j < p; j++) A[i][j] -= f * A[k][j];
                w[i] -= f * w[k];
            }
        }
        for (int k = p-1; k >= 0; k--) {
            for (int i = 0; i < k; i++) {
                w[i] -= A[i][k] * w[k];
            }
        }
        return w;
    }
}
