// Chapter16_Lesson2.java
// Difference Equations and Discrete-Time State-Space Models

public class Chapter16_Lesson2 {

    static double[] simulateScalarDifference(double a, double b, double[] u, double y0) {
        double[] y = new double[u.length + 1];
        y[0] = y0;
        for (int k = 0; k < u.length; k++) {
            y[k + 1] = a * y[k] + b * u[k];
        }
        return y;
    }

    static class SimResult {
        double[][] X;
        double[] Y;
        SimResult(int N, int nx) {
            X = new double[N + 1][nx];
            Y = new double[N];
        }
    }

    static SimResult simulateDiscreteSS(double[][] A, double[] B, double[] C, double D,
                                        double[] u, double[] x0) {
        int N = u.length;
        int nx = x0.length;
        SimResult r = new SimResult(N, nx);

        double[] x = new double[nx];
        System.arraycopy(x0, 0, x, 0, nx);
        System.arraycopy(x, 0, r.X[0], 0, nx);

        for (int k = 0; k < N; k++) {
            double uk = u[k];
            double yk = 0.0;
            for (int i = 0; i < nx; i++) {
                yk += C[i] * x[i];
            }
            yk += D * uk;
            r.Y[k] = yk;

            double[] xNext = new double[nx];
            for (int i = 0; i < nx; i++) {
                double sum = 0.0;
                for (int j = 0; j < nx; j++) {
                    sum += A[i][j] * x[j];
                }
                xNext[i] = sum + B[i] * uk;
            }

            x = xNext;
            System.arraycopy(x, 0, r.X[k + 1], 0, nx);
        }

        return r;
    }

    public static void main(String[] args) {
        int N = 40;

        // Example 1: scalar recursion
        double[] u1 = new double[N];
        for (int k = 0; k < N; k++) u1[k] = 1.0;
        double[] y = simulateScalarDifference(0.85, 0.2, u1, 0.0);

        System.out.println("Scalar recursion (first 10 samples)");
        for (int k = 0; k < 10; k++) {
            System.out.printf("k=%2d  y=% .6f%n", k, y[k]);
        }
        System.out.println();

        // Example 2: companion-form discrete state-space
        // y[k+2] - 1.5 y[k+1] + 0.7 y[k] = u[k]
        double[][] A = {
            {1.5, -0.7},
            {1.0,  0.0}
        };
        double[] B = {1.0, 0.0};
        double[] C = {0.0, 1.0};
        double D = 0.0;

        double[] u2 = new double[N];
        for (int k = 0; k < 10; k++) u2[k] = 1.0;
        double[] x0 = {0.0, 0.0};

        SimResult r = simulateDiscreteSS(A, B, C, D, u2, x0);

        System.out.println("State-space output (first 10 samples)");
        for (int k = 0; k < 10; k++) {
            System.out.printf("k=%2d  y=% .6f  x=[% .6f, % .6f]%n",
                              k, r.Y[k], r.X[k][0], r.X[k][1]);
        }
    }
}
