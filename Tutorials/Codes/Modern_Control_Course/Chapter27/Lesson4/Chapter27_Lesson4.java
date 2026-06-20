/*
Chapter27_Lesson4.java
State-space disturbance rejection using integral action.

Compile and run:
    javac Chapter27_Lesson4.java
    java Chapter27_Lesson4
*/

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

public class Chapter27_Lesson4 {
    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length;
        int n = B[0].length;
        int p = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < p; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    static double[][] scale(double[][] A, double alpha) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = alpha * A[i][j];
            }
        }
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n + i] = 1.0;
        }

        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(aug[row][col]) > Math.abs(aug[pivot][col])) {
                    pivot = row;
                }
            }
            double[] temp = aug[col];
            aug[col] = aug[pivot];
            aug[pivot] = temp;

            double div = aug[col][col];
            if (Math.abs(div) < 1e-12) throw new RuntimeException("Singular matrix");
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= div;

            for (int row = 0; row < n; row++) {
                if (row == col) continue;
                double factor = aug[row][col];
                for (int j = 0; j < 2 * n; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) inv[i][j] = aug[i][n + j];
        }
        return inv;
    }

    static double[][] column(double a, double b, double c) {
        return new double[][]{{a}, {b}, {c}};
    }

    static double[][] controllability(double[][] A, double[][] B) {
        double[][] AB = multiply(A, B);
        double[][] A2B = multiply(A, AB);
        return new double[][]{
                {B[0][0], AB[0][0], A2B[0][0]},
                {B[1][0], AB[1][0], A2B[1][0]},
                {B[2][0], AB[2][0], A2B[2][0]}
        };
    }

    static double[][] desiredPolynomialMatrix(double[][] A) {
        // Desired poles: -2, -2.5, -3
        // p(s) = s^3 + 7.5 s^2 + 18.5 s + 15
        double[][] A2 = multiply(A, A);
        double[][] A3 = multiply(A2, A);
        return add(add(add(A3, scale(A2, 7.5)), scale(A, 18.5)), scale(identity(3), 15.0));
    }

    static double[] ackermannGain(double[][] A, double[][] B) {
        double[][] ctrb = controllability(A, B);
        double[][] phiA = desiredPolynomialMatrix(A);
        double[][] last = new double[][]{{0.0, 0.0, 1.0}};
        double[][] K = multiply(multiply(last, inverse(ctrb)), phiA);
        return K[0];
    }

    static double[] dynamics(double[] xa, double[] K, double d0) {
        double x1 = xa[0];
        double x2 = xa[1];
        double xi = xa[2];
        double u = -(K[0] * x1 + K[1] * x2 + K[2] * xi);
        return new double[]{
                x2,
                -2.0 * x1 - 0.6 * x2 + u + d0,
                x1
        };
    }

    static double[] addScaled(double[] x, double[] dx, double h) {
        return new double[]{x[0] + h * dx[0], x[1] + h * dx[1], x[2] + h * dx[2]};
    }

    public static void main(String[] args) throws IOException {
        double[][] Aaug = new double[][]{
                {0.0, 1.0, 0.0},
                {-2.0, -0.6, 0.0},
                {1.0, 0.0, 0.0}
        };
        double[][] Baug = column(0.0, 1.0, 0.0);
        double[] K = ackermannGain(Aaug, Baug);
        System.out.println("Kaug = " + Arrays.toString(K));

        double d0 = 0.5;
        double h = 0.001;
        double tf = 10.0;
        double[] xa = new double[]{0.4, 0.0, 0.0};

        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter27_Lesson4_java_response.csv"))) {
            out.println("t,x1,x2,xi,u,y");
            int steps = (int) (tf / h);
            for (int k = 0; k <= steps; k++) {
                double t = k * h;
                double u = -(K[0] * xa[0] + K[1] * xa[1] + K[2] * xa[2]);
                out.printf("%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n", t, xa[0], xa[1], xa[2], u, xa[0]);

                double[] k1 = dynamics(xa, K, d0);
                double[] k2 = dynamics(addScaled(xa, k1, 0.5 * h), K, d0);
                double[] k3 = dynamics(addScaled(xa, k2, 0.5 * h), K, d0);
                double[] k4 = dynamics(addScaled(xa, k3, h), K, d0);
                for (int i = 0; i < 3; i++) {
                    xa[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
                }
            }
        }

        System.out.println("Final y = " + xa[0]);
        System.out.println("CSV written to Chapter27_Lesson4_java_response.csv");
    }
}
