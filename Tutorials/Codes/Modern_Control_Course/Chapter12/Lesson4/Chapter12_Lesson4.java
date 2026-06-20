// Chapter12_Lesson4.java
// Modern Control — Chapter 12, Lesson 4
// From-scratch finite-horizon controllability Gramian for a 2-state SISO example.
// No external Java library is required.

public class Chapter12_Lesson4 {
    static double[][] A = {{-1.0, 0.0}, {0.0, -4.0}};
    static double[][] B = {{1.0}, {0.08}};

    static double[][] add(double[][] X, double[][] Y) {
        int r = X.length, c = X[0].length;
        double[][] Z = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) Z[i][j] = X[i][j] + Y[i][j];
        }
        return Z;
    }

    static double[][] scale(double[][] X, double a) {
        int r = X.length, c = X[0].length;
        double[][] Z = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) Z[i][j] = a * X[i][j];
        }
        return Z;
    }

    static double[][] mul(double[][] X, double[][] Y) {
        int r = X.length, c = Y[0].length, inner = Y.length;
        double[][] Z = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                for (int k = 0; k < inner; k++) Z[i][j] += X[i][k] * Y[k][j];
            }
        }
        return Z;
    }

    static double[][] transpose(double[][] X) {
        int r = X.length, c = X[0].length;
        double[][] Z = new double[c][r];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) Z[j][i] = X[i][j];
        }
        return Z;
    }

    static double[][] gramianDerivative(double[][] W) {
        // dW/ds = A W + W A^T + B B^T, W(0)=0.
        return add(add(mul(A, W), mul(W, transpose(A))), mul(B, transpose(B)));
    }

    static double[][] rk4Step(double[][] W, double h) {
        double[][] k1 = gramianDerivative(W);
        double[][] k2 = gramianDerivative(add(W, scale(k1, h / 2.0)));
        double[][] k3 = gramianDerivative(add(W, scale(k2, h / 2.0)));
        double[][] k4 = gramianDerivative(add(W, scale(k3, h)));
        return add(W, scale(add(add(k1, scale(add(k2, k3), 2.0)), k4), h / 6.0));
    }

    static double[][] finiteHorizonGramian(double T, int steps) {
        double h = T / steps;
        double[][] W = {{0.0, 0.0}, {0.0, 0.0}};
        for (int i = 0; i < steps; i++) W = rk4Step(W, h);
        // Symmetrize to remove tiny numerical asymmetry.
        double avg = 0.5 * (W[0][1] + W[1][0]);
        W[0][1] = avg;
        W[1][0] = avg;
        return W;
    }

    static double[] eigenvalues2x2Symmetric(double[][] W) {
        double a = W[0][0], b = W[0][1], d = W[1][1];
        double tr = a + d;
        double disc = Math.sqrt((a - d) * (a - d) + 4.0 * b * b);
        return new double[]{0.5 * (tr - disc), 0.5 * (tr + disc)};
    }

    static double minimumEnergy(double[][] W, double[] delta) {
        double det = W[0][0] * W[1][1] - W[0][1] * W[1][0];
        double inv00 = W[1][1] / det;
        double inv01 = -W[0][1] / det;
        double inv10 = -W[1][0] / det;
        double inv11 = W[0][0] / det;
        return delta[0] * (inv00 * delta[0] + inv01 * delta[1])
             + delta[1] * (inv10 * delta[0] + inv11 * delta[1]);
    }

    static void printMatrix(String name, double[][] X) {
        System.out.println(name + " =");
        for (double[] row : X) {
            for (double v : row) System.out.printf("%14.8f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double T = 2.0;
        double[][] W = finiteHorizonGramian(T, 4000);
        printMatrix("Wc(T)", W);
        double[] lambda = eigenvalues2x2Symmetric(W);
        System.out.printf("Eigenvalues: %.10f %.10f%n", lambda[0], lambda[1]);
        System.out.printf("Condition number: %.6f%n", lambda[1] / lambda[0]);
        System.out.printf("Energy to reach e1: %.8f%n", minimumEnergy(W, new double[]{1.0, 0.0}));
        System.out.printf("Energy to reach e2: %.8f%n", minimumEnergy(W, new double[]{0.0, 1.0}));
        System.out.printf("Best-axis energy: %.8f%n", 1.0 / lambda[1]);
        System.out.printf("Worst-axis energy: %.8f%n", 1.0 / lambda[0]);
    }
}
