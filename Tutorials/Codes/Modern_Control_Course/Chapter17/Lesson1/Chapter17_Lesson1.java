/*
Chapter17_Lesson1.java

Observable Canonical Form (OCF) construction for a SISO transfer function.

Compile:
    javac Chapter17_Lesson1.java
Run:
    java Chapter17_Lesson1

This version uses plain Java arrays. For larger numerical systems, use EJML,
Apache Commons Math, or ND4J.
*/

public class Chapter17_Lesson1 {
    static class StateSpace {
        double[][] A;
        double[] B;
        double[] C;
        double D;

        StateSpace(double[][] A, double[] B, double[] C, double D) {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
        }
    }

    static StateSpace observableCanonicalForm(double[] denInput, double[] numInput) {
        double[] den = denInput.clone();
        double[] num = numInput.clone();

        if (den.length < 2 || Math.abs(den[0]) < 1e-14) {
            throw new IllegalArgumentException("Denominator leading coefficient must be nonzero.");
        }

        double leading = den[0];
        for (int i = 0; i < den.length; i++) {
            den[i] /= leading;
        }
        for (int i = 0; i < num.length; i++) {
            num[i] /= leading;
        }

        int n = den.length - 1;
        if (num.length > n + 1) {
            throw new IllegalArgumentException("Expected a proper transfer function.");
        }

        double[] numPad = new double[n + 1];
        int offset = n + 1 - num.length;
        for (int i = 0; i < num.length; i++) {
            numPad[offset + i] = num[i];
        }

        double D = numPad[0];
        double[] remainder = new double[n + 1];
        for (int i = 0; i <= n; i++) {
            remainder[i] = numPad[i] - D * den[i];
        }

        double[][] A = new double[n][n];
        for (int i = 1; i < n; i++) {
            A[i][i - 1] = 1.0;
        }
        for (int i = 0; i < n; i++) {
            A[i][n - 1] = -den[n - i];
        }

        double[] B = new double[n];
        for (int i = 0; i < n; i++) {
            B[i] = remainder[n - i];
        }

        double[] C = new double[n];
        C[n - 1] = 1.0;

        return new StateSpace(A, B, C, D);
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length;
        int p = B.length;
        int m = B[0].length;
        double[][] C = new double[n][m];

        for (int i = 0; i < n; i++) {
            for (int k = 0; k < p; k++) {
                for (int j = 0; j < m; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) {
            I[i][i] = 1.0;
        }
        return I;
    }

    static double[][] observabilityMatrix(double[][] A, double[] C) {
        int n = A.length;
        double[][] O = new double[n][n];
        double[][] Ak = identity(n);

        for (int row = 0; row < n; row++) {
            for (int j = 0; j < n; j++) {
                double value = 0.0;
                for (int k = 0; k < n; k++) {
                    value += C[k] * Ak[k][j];
                }
                O[row][j] = value;
            }
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static int rankGaussian(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            System.arraycopy(input[i], 0, M[i], 0, cols);
        }

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) {
                    pivot = i;
                }
            }
            if (Math.abs(M[pivot][c]) < tol) {
                continue;
            }
            double[] tmp = M[pivot];
            M[pivot] = M[r];
            M[r] = tmp;

            double div = M[r][c];
            for (int j = c; j < cols; j++) {
                M[r][j] /= div;
            }

            for (int i = 0; i < rows; i++) {
                if (i == r) {
                    continue;
                }
                double factor = M[i][c];
                for (int j = c; j < cols; j++) {
                    M[i][j] -= factor * M[r][j];
                }
            }
            r++;
        }
        return r;
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

    static void printVector(String name, double[] v) {
        System.out.print(name + " = [ ");
        for (double x : v) {
            System.out.printf("%10.6f ", x);
        }
        System.out.println("]");
    }

    public static void main(String[] args) {
        // G(s) = (2 s^2 + 5 s + 3) / (s^3 + 4 s^2 + 6 s + 8)
        double[] den = {1.0, 4.0, 6.0, 8.0};
        double[] num = {2.0, 5.0, 3.0};

        StateSpace sys = observableCanonicalForm(den, num);

        printMatrix("A_o", sys.A);
        printVector("B_o", sys.B);
        printVector("C_o", sys.C);
        System.out.println("D_o = " + sys.D);

        double[][] O = observabilityMatrix(sys.A, sys.C);
        printMatrix("O_o", O);
        System.out.println("rank(O_o) = " + rankGaussian(O, 1e-10));
    }
}
