/*
Chapter30_Lesson3.java
Pure Java state-space utilities for small educational models.

Production libraries to consider:
    EJML, Apache Commons Math, Hipparchus, ojAlgo
*/

import java.util.Arrays;

public class Chapter30_Lesson3 {
    static double[][] matMul(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, r = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                double s = 0.0;
                for (int k = 0; k < r; k++) s += A[i][k] * B[k][j];
                C[i][j] = s;
            }
        }
        return C;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scale(double a, double[][] A) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = a * A[i][j];
        return C;
    }

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] hcat(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length + B[0].length];
        for (int i = 0; i < A.length; i++) {
            System.arraycopy(A[i], 0, C[i], 0, A[0].length);
            System.arraycopy(B[i], 0, C[i], A[0].length, B[0].length);
        }
        return C;
    }

    static double[][] vcat(double[][] A, double[][] B) {
        double[][] C = new double[A.length + B.length][A[0].length];
        for (int i = 0; i < A.length; i++) C[i] = Arrays.copyOf(A[i], A[i].length);
        for (int i = 0; i < B.length; i++) C[A.length + i] = Arrays.copyOf(B[i], B[i].length);
        return C;
    }

    static double[][] controllability(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Ctrb = B;
        double[][] Ak = eye(n);
        for (int k = 1; k < n; k++) {
            Ak = matMul(A, Ak);
            Ctrb = hcat(Ctrb, matMul(Ak, B));
        }
        return Ctrb;
    }

    static double[][] observability(double[][] A, double[][] C) {
        int n = A.length;
        double[][] Obsv = C;
        double[][] Ak = eye(n);
        for (int k = 1; k < n; k++) {
            Ak = matMul(Ak, A);
            Obsv = vcat(Obsv, matMul(C, Ak));
        }
        return Obsv;
    }

    static double det2(double[][] M) {
        return M[0][0] * M[1][1] - M[0][1] * M[1][0];
    }

    static double[][] inv2(double[][] M) {
        double det = det2(M);
        if (Math.abs(det) < 1e-12) throw new RuntimeException("Singular 2x2 matrix");
        return new double[][]{{M[1][1] / det, -M[0][1] / det}, {-M[1][0] / det, M[0][0] / det}};
    }

    static double[][] ackermann2x1(double[][] A, double[][] B, double pole1, double pole2) {
        // Desired polynomial: s^2 + a1 s + a0.
        double a1 = -(pole1 + pole2);
        double a0 = pole1 * pole2;
        double[][] A2 = matMul(A, A);
        double[][] phiA = matAdd(matAdd(A2, scale(a1, A)), scale(a0, eye(2)));
        double[][] CtrbInv = inv2(controllability(A, B));
        double[][] eT = new double[][]{{0.0, 1.0}};
        return matMul(matMul(eT, CtrbInv), phiA);
    }

    static double[] closedLoopDerivative(double[][] A, double[][] B, double[][] K, double[] x) {
        double[][] Acl = matAdd(A, scale(-1.0, matMul(B, K)));
        return new double[]{Acl[0][0] * x[0] + Acl[0][1] * x[1], Acl[1][0] * x[0] + Acl[1][1] * x[1]};
    }

    static double[] rk4(double[][] A, double[][] B, double[][] K, double[] x, double h) {
        double[] k1 = closedLoopDerivative(A, B, K, x);
        double[] k2 = closedLoopDerivative(A, B, K, new double[]{x[0] + 0.5 * h * k1[0], x[1] + 0.5 * h * k1[1]});
        double[] k3 = closedLoopDerivative(A, B, K, new double[]{x[0] + 0.5 * h * k2[0], x[1] + 0.5 * h * k2[1]});
        double[] k4 = closedLoopDerivative(A, B, K, new double[]{x[0] + h * k3[0], x[1] + h * k3[1]});
        return new double[]{x[0] + h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6.0,
                x[1] + h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6.0};
    }

    static void print(String title, double[][] A) {
        System.out.println(title);
        for (double[] row : A) System.out.println(Arrays.toString(row));
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -0.35}};
        double[][] B = {{0.0}, {1.0}};
        double[][] C = {{1.0, 0.0}};

        print("Controllability matrix", controllability(A, B));
        print("Observability matrix", observability(A, C));

        double[][] K = ackermann2x1(A, B, -2.0, -3.0);
        print("K from Ackermann", K);

        double[] x = {1.0, 0.0};
        double h = 0.01;
        for (int i = 0; i <= 500; i++) {
            if (i % 100 == 0) System.out.printf("t=%.2f, x=[%.6f, %.6f]%n", i * h, x[0], x[1]);
            x = rk4(A, B, K, x, h);
        }
    }
}
