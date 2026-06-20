// Chapter16_Lesson4.java
// Continuous–Discrete Conversions: Zero-Order Hold, Exact Discretization
// Pure Java implementation (from-scratch matrix utilities and expm series for small systems)
// For production, you may use EJML or Apache Commons Math.

import java.util.Arrays;

public class Chapter16_Lesson4 {

    static class DiscreteModel {
        double[][] Ad;
        double[][] Bd;
        DiscreteModel(double[][] Ad, double[][] Bd) { this.Ad = Ad; this.Bd = Bd; }
    }

    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] eye(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] copy(double[][] A) {
        double[][] B = zeros(A.length, A[0].length);
        for (int i = 0; i < A.length; i++) System.arraycopy(A[i], 0, B[i], 0, A[0].length);
        return B;
    }

    static double[][] add(double[][] A, double[][] B) {
        int r = A.length, c = A[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                R[i][j] = A[i][j] + B[i][j];
        return R;
    }

    static double[][] scale(double[][] A, double s) {
        int r = A.length, c = A[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                R[i][j] = s * A[i][j];
        return R;
    }

    static double[][] mul(double[][] A, double[][] B) {
        int r = A.length, n = A[0].length, c = B[0].length;
        double[][] R = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int k = 0; k < n; k++) {
                double aik = A[i][k];
                for (int j = 0; j < c; j++) {
                    R[i][j] += aik * B[k][j];
                }
            }
        }
        return R;
    }

    static double[][] blockAugment(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length;
        double[][] M = zeros(n + m, n + m);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                M[i][j] = A[i][j];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                M[i][n + j] = B[i][j];
        return M;
    }

    // Exponential by truncated Taylor series; good for small Ts and small matrices (teaching/demo)
    static double[][] expmSeries(double[][] A, int terms) {
        int n = A.length;
        double[][] result = eye(n);
        double[][] power = eye(n);
        double factorial = 1.0;

        for (int k = 1; k <= terms; k++) {
            power = mul(power, A);
            factorial *= k;
            result = add(result, scale(power, 1.0 / factorial));
        }
        return result;
    }

    static DiscreteModel exactDiscretizeZOH(double[][] A, double[][] B, double Ts) {
        int n = A.length;
        int m = B[0].length;

        double[][] M = blockAugment(A, B);
        double[][] Md = expmSeries(scale(M, Ts), 40);

        double[][] Ad = zeros(n, n);
        double[][] Bd = zeros(n, m);

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) Ad[i][j] = Md[i][j];
            for (int j = 0; j < m; j++) Bd[i][j] = Md[i][n + j];
        }
        return new DiscreteModel(Ad, Bd);
    }

    static DiscreteModel eulerDiscretize(double[][] A, double[][] B, double Ts) {
        int n = A.length;
        double[][] Ad = add(eye(n), scale(A, Ts));
        double[][] Bd = scale(B, Ts);
        return new DiscreteModel(Ad, Bd);
    }

    static double[] matVec(double[][] A, double[] x) {
        int r = A.length, c = A[0].length;
        double[] y = new double[r];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) y[i] += A[i][j] * x[j];
        }
        return y;
    }

    static double[] vecAdd(double[] a, double[] b) {
        double[] r = new double[a.length];
        for (int i = 0; i < a.length; i++) r[i] = a[i] + b[i];
        return r;
    }

    static double[] scaleVec(double[] a, double s) {
        double[] r = new double[a.length];
        for (int i = 0; i < a.length; i++) r[i] = s * a[i];
        return r;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name + " =");
        for (double[] row : A) System.out.println(Arrays.toString(row));
        System.out.println();
    }

    public static void main(String[] args) {
        // Mass-spring-damper example
        double m = 1.0, c = 0.6, k = 4.0, b = 1.0;
        double[][] A = {
            {0.0, 1.0},
            {-k / m, -c / m}
        };
        double[][] B = {
            {0.0},
            {b / m}
        };
        double[][] C = {
            {1.0, 0.0}
        };
        double[][] D = {
            {0.0}
        };
        double Ts = 0.1;

        DiscreteModel exact = exactDiscretizeZOH(A, B, Ts);
        DiscreteModel euler = eulerDiscretize(A, B, Ts);

        printMatrix("Ad_exact", exact.Ad);
        printMatrix("Bd_exact", exact.Bd);
        printMatrix("Ad_euler", euler.Ad);
        printMatrix("Bd_euler", euler.Bd);

        int N = 60;
        double[] xExact = {0.0, 0.0};
        double[] xEuler = {0.0, 0.0};

        System.out.println("k,t,u,yExact,yEuler");
        for (int kStep = 0; kStep < N; kStep++) {
            double u = (kStep >= 5) ? 1.0 : 0.0;

            double yExact = matVec(C, xExact)[0] + D[0][0] * u;
            double yEuler = matVec(C, xEuler)[0] + D[0][0] * u;
            System.out.printf("%d,%.3f,%.1f,%.6f,%.6f%n", kStep, kStep * Ts, u, yExact, yEuler);

            xExact = vecAdd(matVec(exact.Ad, xExact), scaleVec(new double[]{exact.Bd[0][0], exact.Bd[1][0]}, u));
            xEuler = vecAdd(matVec(euler.Ad, xEuler), scaleVec(new double[]{euler.Bd[0][0], euler.Bd[1][0]}, u));
        }
    }
}
