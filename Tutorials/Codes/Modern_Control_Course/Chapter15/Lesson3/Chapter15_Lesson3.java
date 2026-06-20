// Chapter15_Lesson3.java
// Finite-horizon observability Gramian and qualitative coordinate-sensor scoring.
// From-scratch implementation for small teaching examples.
//
// Compile and run:
//   javac Chapter15_Lesson3.java
//   java Chapter15_Lesson3

import java.util.Arrays;

public class Chapter15_Lesson3 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] transpose(double[][] A) {
        int r = A.length;
        int c = A[0].length;
        double[][] T = zeros(c, r);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] add(double[][] A, double[][] B) {
        int r = A.length;
        int c = A[0].length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        int r = A.length;
        int c = A[0].length;
        double[][] B = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                B[i][j] = s * A[i][j];
        return B;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int c = B[0].length;
        int inner = B.length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int k = 0; k < inner; k++)
                for (int j = 0; j < c; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] sensorMatrix(int[] sensors, int n) {
        double[][] C = zeros(sensors.length, n);
        for (int r = 0; r < sensors.length; r++) {
            C[r][sensors[r]] = 1.0;
        }
        return C;
    }

    static double[][] gramianRhs(double[][] A, double[][] W, double[][] Q) {
        double[][] AT = transpose(A);
        return add(add(multiply(AT, W), multiply(W, A)), Q);
    }

    static double[][] finiteHorizonGramian(double[][] A, double[][] C, double T, int steps) {
        int n = A.length;
        double[][] W = zeros(n, n);
        double[][] Q = multiply(transpose(C), C);
        double h = T / steps;

        for (int s = 0; s < steps; s++) {
            double[][] k1 = gramianRhs(A, W, Q);
            double[][] k2 = gramianRhs(A, add(W, scale(k1, 0.5 * h)), Q);
            double[][] k3 = gramianRhs(A, add(W, scale(k2, 0.5 * h)), Q);
            double[][] k4 = gramianRhs(A, add(W, scale(k3, h)), Q);

            double[][] increment = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
            W = add(W, scale(increment, h / 6.0));
        }

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double avg = 0.5 * (W[i][j] + W[j][i]);
                W[i][j] = avg;
                W[j][i] = avg;
            }
        }

        return W;
    }

    static double trace(double[][] A) {
        double s = 0.0;
        for (int i = 0; i < A.length; i++) s += A[i][i];
        return s;
    }

    static double det3(double[][] A) {
        return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
             - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
             + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    }

    static double regularizedLogDet3(double[][] W, double eps) {
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; i++) R[i] = Arrays.copyOf(W[i], 3);
        for (int i = 0; i < 3; i++) R[i][i] += eps;
        double d = det3(R);
        if (d <= 0.0) return Double.NEGATIVE_INFINITY;
        return Math.log(d);
    }

    static void printScore(String name, double[][] W) {
        System.out.printf(
            "%8s  trace=%12.6f  logdet_eps=%12.6f  det_eps=%12.6e%n",
            name, trace(W), regularizedLogDet3(W, 1e-8),
            Math.exp(regularizedLogDet3(W, 1e-8))
        );
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {-2.0, -0.45, 0.8},
            {0.0, -0.7, -1.25}
        };

        int n = 3;
        double T = 6.0;
        int steps = 4000;

        System.out.println("Single-sensor scores");
        for (int i = 0; i < n; i++) {
            double[][] C = sensorMatrix(new int[] {i}, n);
            double[][] W = finiteHorizonGramian(A, C, T, steps);
            printScore("x" + (i + 1), W);
        }

        System.out.println("\nTwo-sensor scores");
        double bestScore = Double.NEGATIVE_INFINITY;
        String bestName = "";
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double[][] C = sensorMatrix(new int[] {i, j}, n);
                double[][] W = finiteHorizonGramian(A, C, T, steps);
                String name = "x" + (i + 1) + ",x" + (j + 1);
                double score = regularizedLogDet3(W, 1e-8);
                printScore(name, W);
                if (score > bestScore) {
                    bestScore = score;
                    bestName = name;
                }
            }
        }

        System.out.println("\nBest qualitative two-sensor set by log-det: " + bestName);
    }
}
