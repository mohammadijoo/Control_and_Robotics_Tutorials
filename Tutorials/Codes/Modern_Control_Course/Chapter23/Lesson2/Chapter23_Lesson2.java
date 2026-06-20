/*
Chapter23_Lesson2.java
Pole Assignment via Controllable Canonical Form (CCF)

This implementation uses coefficient matching for a SISO system already in CCF.
Compile:
    javac Chapter23_Lesson2.java
Run:
    java Chapter23_Lesson2
*/

import java.util.Arrays;

public class Chapter23_Lesson2 {

    static double[][] companionMatrix(double[] aAscending) {
        int n = aAscending.length;
        double[][] A = new double[n][n];
        for (int i = 0; i < n - 1; i++) {
            A[i][i + 1] = 1.0;
        }
        for (int j = 0; j < n; j++) {
            A[n - 1][j] = -aAscending[j];
        }
        return A;
    }

    static double[] inputVector(int n) {
        double[] b = new double[n];
        b[n - 1] = 1.0;
        return b;
    }

    static double[] polynomialFromRealRoots(double[] roots) {
        // Descending coefficients for product (s-root_i)
        double[] coeff = {1.0};
        for (double r : roots) {
            double[] next = new double[coeff.length + 1];
            for (int i = 0; i < coeff.length; i++) {
                next[i] += coeff[i];
                next[i + 1] += -r * coeff[i];
            }
            coeff = next;
        }
        return coeff;
    }

    static double[] desiredCoefficientsAscending(double[] desiredPoles) {
        double[] desc = polynomialFromRealRoots(desiredPoles);
        int n = desiredPoles.length;
        double[] asc = new double[n];
        for (int i = 0; i < n; i++) {
            asc[i] = desc[n - i];
        }
        return asc;
    }

    static double[] ccfGain(double[] openCoeffsAscending, double[] desiredPoles) {
        double[] alpha = desiredCoefficientsAscending(desiredPoles);
        if (alpha.length != openCoeffsAscending.length) {
            throw new IllegalArgumentException("System order and number of desired poles differ.");
        }
        double[] K = new double[alpha.length];
        for (int i = 0; i < alpha.length; i++) {
            K[i] = alpha[i] - openCoeffsAscending[i];
        }
        return K;
    }

    static double[][] closedLoopMatrix(double[][] A, double[] b, double[] K) {
        int n = A.length;
        double[][] Acl = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Acl[i][j] = A[i][j] - b[i] * K[j];
            }
        }
        return Acl;
    }

    static void printMatrix(double[][] A, String name) {
        System.out.println(name + " =");
        for (double[] row : A) {
            System.out.println(Arrays.toString(row));
        }
    }

    static void printVector(double[] v, String name) {
        System.out.println(name + " = " + Arrays.toString(v));
    }

    public static void main(String[] args) {
        // p(s)=s^3+6s^2+11s+6; desired poles {-4,-5,-6}
        double[] a = {6.0, 11.0, 6.0};             // [a0,a1,a2]
        double[] desiredPoles = {-4.0, -5.0, -6.0};

        double[][] Ac = companionMatrix(a);
        double[] bc = inputVector(a.length);
        double[] Kc = ccfGain(a, desiredPoles);
        double[][] Acl = closedLoopMatrix(Ac, bc, Kc);

        printMatrix(Ac, "A_c");
        printVector(bc, "b_c");
        printVector(Kc, "K_c");
        printMatrix(Acl, "A_c - b_c K_c");

        System.out.println("\nExpected closed-loop polynomial:");
        System.out.println("s^3 + 15 s^2 + 74 s + 120 = (s+4)(s+5)(s+6)");
    }
}
