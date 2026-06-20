/*
Chapter28_Lesson3.java
Modern Control — Chapter 28, Lesson 3
State and control weighting matrices Q and R as performance descriptors.

This standalone Java example computes a finite-horizon weighted cost
for xdot = (A - B K)x, u = -Kx using RK4 integration.
*/

public class Chapter28_Lesson3 {
    static double[] matVec(double[][] A, double[] x) {
        return new double[] {
            A[0][0] * x[0] + A[0][1] * x[1],
            A[1][0] * x[0] + A[1][1] * x[1]
        };
    }

    static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1]};
    }

    static double[] scale(double c, double[] x) {
        return new double[] {c * x[0], c * x[1]};
    }

    static double[] rhs(double[][] Acl, double[] x) {
        return matVec(Acl, x);
    }

    static double[] rk4Step(double[][] Acl, double[] x, double h) {
        double[] k1 = rhs(Acl, x);
        double[] k2 = rhs(Acl, add(x, scale(0.5 * h, k1)));
        double[] k3 = rhs(Acl, add(x, scale(0.5 * h, k2)));
        double[] k4 = rhs(Acl, add(x, scale(h, k3)));
        return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
    }

    static double quad(double[] x, double[][] Q) {
        return x[0] * (Q[0][0] * x[0] + Q[0][1] * x[1])
             + x[1] * (Q[1][0] * x[0] + Q[1][1] * x[1]);
    }

    static double input(double[] K, double[] x) {
        return -(K[0] * x[0] + K[1] * x[1]);
    }

    static boolean isPsd2x2(double[][] Q) {
        double det = Q[0][0] * Q[1][1] - Q[0][1] * Q[1][0];
        return Q[0][0] >= -1e-12 && Q[1][1] >= -1e-12 && det >= -1e-12;
    }

    public static void main(String[] args) {
        double[][] A = {{0.0, 1.0}, {-2.0, -0.4}};
        double[] B = {0.0, 1.0};
        double[] K = {3.0, 2.2};

        double[][] Acl = new double[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                Acl[i][j] = A[i][j] - B[i] * K[j];
            }
        }

        // Bryson-style tolerances: x1_max=1, x2_max=2, u_max=0.5.
        double[][] Q = {{1.0, 0.0}, {0.0, 1.0 / 4.0}};
        double R = 1.0 / (0.5 * 0.5);

        if (!isPsd2x2(Q)) {
            throw new IllegalArgumentException("Q must be positive semidefinite.");
        }
        if (R <= 0.0) {
            throw new IllegalArgumentException("R must be positive definite.");
        }

        double[] x = {1.0, 0.0};
        double h = 0.001;
        double tf = 8.0;
        int steps = (int) Math.round(tf / h);
        double J = 0.0;

        double fOld = quad(x, Q) + R * input(K, x) * input(K, x);
        for (int k = 0; k < steps; ++k) {
            double[] xNext = rk4Step(Acl, x, h);
            double uNext = input(K, xNext);
            double fNew = quad(xNext, Q) + R * uNext * uNext;
            J += 0.5 * h * (fOld + fNew);
            x = xNext;
            fOld = fNew;
        }

        System.out.println("Q = [[1, 0], [0, 0.25]]");
        System.out.println("R = " + R);
        System.out.println("Finite-horizon weighted cost J_T = " + J);
    }
}
