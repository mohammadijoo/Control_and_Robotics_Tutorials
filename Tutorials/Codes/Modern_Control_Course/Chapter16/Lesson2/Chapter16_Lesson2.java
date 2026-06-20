/*
Chapter16_Lesson2.java

Construction of Controllable Canonical Form (CCF) from SISO transfer-function data.

Compile:
    javac Chapter16_Lesson2.java
Run:
    java Chapter16_Lesson2
*/

import java.util.Arrays;

public class Chapter16_Lesson2 {
    static class StateSpace {
        double[][] A;
        double[][] B;
        double[][] C;
        double D;

        StateSpace(double[][] A, double[][] B, double[][] C, double D) {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
        }
    }

    static double[] trimLeadingZeros(double[] coeffs) {
        if (coeffs.length == 0) {
            throw new IllegalArgumentException("Coefficient vector cannot be empty.");
        }
        int i = 0;
        while (i + 1 < coeffs.length && Math.abs(coeffs[i]) < 1e-12) {
            i++;
        }
        return Arrays.copyOfRange(coeffs, i, coeffs.length);
    }

    static StateSpace controllableCanonicalForm(double[] numerator, double[] denominator) {
        double[] num = trimLeadingZeros(numerator);
        double[] den = trimLeadingZeros(denominator);

        if (Math.abs(den[0]) < 1e-12) {
            throw new IllegalArgumentException("Leading denominator coefficient must be nonzero.");
        }

        double leading = den[0];
        for (int i = 0; i < den.length; i++) den[i] /= leading;
        for (int i = 0; i < num.length; i++) num[i] /= leading;

        int n = den.length - 1;
        if (n < 1) {
            throw new IllegalArgumentException("Denominator degree must be at least one.");
        }
        if (num.length > n + 1) {
            throw new IllegalArgumentException("Improper transfer function: numerator degree exceeds denominator degree.");
        }

        double[] numFull = new double[n + 1];
        int offset = (n + 1) - num.length;
        for (int i = 0; i < num.length; i++) {
            numFull[offset + i] = num[i];
        }

        double dFeed = numFull[0];
        double[] rem = new double[n + 1];
        for (int i = 0; i <= n; i++) {
            rem[i] = numFull[i] - dFeed * den[i];
        }

        double[][] A = new double[n][n];
        for (int i = 0; i < n - 1; i++) {
            A[i][i + 1] = 1.0;
        }
        for (int j = 0; j < n; j++) {
            A[n - 1][j] = -den[n - j];
        }

        double[][] B = new double[n][1];
        B[n - 1][0] = 1.0;

        double[][] C = new double[1][n];
        for (int j = 0; j < n; j++) {
            C[0][j] = rem[n - j];
        }

        return new StateSpace(A, B, C, dFeed);
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double v : row) {
                System.out.printf("%12.6f ", v);
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        // G(s) = (2 s^2 + 5 s + 3)/(s^3 + 4 s^2 + 6 s + 8)
        double[] num = {2, 5, 3};
        double[] den = {1, 4, 6, 8};

        StateSpace ss = controllableCanonicalForm(num, den);

        printMatrix("A", ss.A);
        printMatrix("B", ss.B);
        printMatrix("C", ss.C);
        System.out.printf("D = %.6f%n", ss.D);
    }
}
