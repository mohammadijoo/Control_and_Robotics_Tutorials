// Chapter23_Lesson5.java
// Numerical Sensitivity and Conditioning in SISO Pole Placement
//
// Plain Java implementation for a 3rd-order SISO system.
// For production numerical work, use EJML, Apache Commons Math, or JBLAS.

import java.util.Arrays;

public class Chapter23_Lesson5 {
    static double[][] matMul(double[][] A, double[][] B) {
        int m = A.length, n = B[0].length, p = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int k = 0; k < p; k++)
                for (int j = 0; j < n; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scalarMul(double a, double[][] A) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = a * A[i][j];
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2*n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, aug[i], 0, n);
            aug[i][n+i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col+1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            double[] tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;
            double piv = aug[col][col];
            if (Math.abs(piv) < 1e-14) throw new RuntimeException("Singular matrix.");
            for (int j = 0; j < 2*n; j++) aug[col][j] /= piv;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2*n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            System.arraycopy(aug[i], n, inv[i], 0, n);
        return inv;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] C = new double[n][n];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            double[][] col = matMul(Ak, B);
            for (int i = 0; i < n; i++) C[i][k] = col[i][0];
            Ak = matMul(A, Ak);
        }
        return C;
    }

    static double[][] polynomialMatrix(double[][] A, double[] coeffs) {
        int n = A.length;
        double[][] P = new double[n][n];
        for (double c : coeffs) {
            P = matAdd(matMul(P, A), scalarMul(c, identity(n)));
        }
        return P;
    }

    static double[] polyFromRoots3(double r1, double r2, double r3) {
        double a2 = -(r1 + r2 + r3);
        double a1 = r1*r2 + r1*r3 + r2*r3;
        double a0 = -r1*r2*r3;
        return new double[]{1.0, a2, a1, a0};
    }

    static double[][] ackermannGain(double[][] A, double[][] B, double[] coeffs) {
        int n = A.length;
        double[][] Cinv = inverse(controllabilityMatrix(A, B));
        double[][] pA = polynomialMatrix(A, coeffs);
        double[][] eT = new double[1][n];
        eT[0][n-1] = 1.0;
        return matMul(matMul(eT, Cinv), pA);
    }

    static double determinant3(double[][] A) {
        return A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
             - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
             + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name + " =");
        for (double[] row : A) System.out.println(Arrays.toString(row));
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {-2.0, -3.0, -4.0}
        };
        double[][] B = {
            {0.0},
            {0.0},
            {1.0}
        };

        double[] coeffs = polyFromRoots3(-4.0, -5.0, -6.0);
        double[][] C = controllabilityMatrix(A, B);
        double[][] K = ackermannGain(A, B, coeffs);
        double[][] M = matAdd(A, scalarMul(-1.0, matMul(B, K)));

        System.out.println("Desired polynomial coefficients:");
        System.out.println(Arrays.toString(coeffs));
        printMatrix("Controllability matrix C", C);
        System.out.println("det(C) = " + determinant3(C));
        printMatrix("K", K);
        printMatrix("A - B K", M);

        System.out.println("\nFor exact pole verification, compare the characteristic polynomial");
        System.out.println("of A-BK with the desired polynomial. Production Java code should use");
        System.out.println("EJML or Apache Commons Math for SVD-based condition numbers and eigenvalues.");
    }
}
