/*
Chapter10_Lesson1.java

Run:
    javac Chapter10_Lesson1.java
    java Chapter10_Lesson1

This program implements finite-horizon state steering for the double integrator
using basic arrays only. It solves a 2-by-2 linear system for the minimum-norm
piecewise-constant input sequence.
*/

public class Chapter10_Lesson1 {
    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int p = A[0].length;
        int n = B[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < n; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[] multiply(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < x.length; j++) {
                y[i] += A[i][j] * x[j];
            }
        }
        return y;
    }

    static double[][] matrixPower(double[][] A, int p) {
        double[][] R = {{1.0, 0.0}, {0.0, 1.0}};
        for (int i = 0; i < p; i++) {
            R = multiply(R, A);
        }
        return R;
    }

    static double[] solve2x2(double[][] A, double[] b) {
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Singular 2-by-2 system.");
        }
        return new double[] {
            ( b[0] * A[1][1] - A[0][1] * b[1]) / det,
            (-b[0] * A[1][0] + A[0][0] * b[1]) / det
        };
    }

    static double norm(double[] v) {
        double s = 0.0;
        for (double value : v) {
            s += value * value;
        }
        return Math.sqrt(s);
    }

    public static void main(String[] args) {
        double T = 2.0;
        int N = 60;
        double dt = T / N;

        double[][] Phi = {{1.0, dt}, {0.0, 1.0}};
        double[] Gamma = {0.5 * dt * dt, dt};

        double[] x0 = {0.0, 0.0};
        double[] xf = {1.0, 0.0};

        double[][] S = new double[2][N];
        for (int k = 0; k < N; k++) {
            double[][] P = matrixPower(Phi, N - 1 - k);
            double[] col = multiply(P, Gamma);
            S[0][k] = col[0];
            S[1][k] = col[1];
        }

        double[][] PhiN = matrixPower(Phi, N);
        double[] freeResponse = multiply(PhiN, x0);
        double[] targetShift = {xf[0] - freeResponse[0], xf[1] - freeResponse[1]};

        double[][] gram = new double[2][2];
        for (int k = 0; k < N; k++) {
            gram[0][0] += S[0][k] * S[0][k];
            gram[0][1] += S[0][k] * S[1][k];
            gram[1][0] += S[1][k] * S[0][k];
            gram[1][1] += S[1][k] * S[1][k];
        }

        double[] lambda = solve2x2(gram, targetShift);

        double[] u = new double[N];
        for (int k = 0; k < N; k++) {
            u[k] = S[0][k] * lambda[0] + S[1][k] * lambda[1];
        }

        double[][] x = new double[2][N + 1];
        x[0][0] = x0[0];
        x[1][0] = x0[1];

        for (int k = 0; k < N; k++) {
            x[0][k + 1] = Phi[0][0] * x[0][k] + Phi[0][1] * x[1][k] + Gamma[0] * u[k];
            x[1][k + 1] = Phi[1][0] * x[0][k] + Phi[1][1] * x[1][k] + Gamma[1] * u[k];
        }

        double[] error = {x[0][N] - xf[0], x[1][N] - xf[1]};
        double energy = 0.0;
        for (double uk : u) {
            energy += dt * uk * uk;
        }

        System.out.printf("Requested final state: [%.12f, %.12f]%n", xf[0], xf[1]);
        System.out.printf("Achieved final state:  [%.12f, %.12f]%n", x[0][N], x[1][N]);
        System.out.printf("Final steering error norm: %.6e%n", norm(error));
        System.out.printf("Input energy approximation: %.12f%n", energy);
    }
}
