// Chapter20_Lesson1.java
// Minimal realization tests for continuous-time LTI systems.
// Compile: javac Chapter20_Lesson1.java
// Run:     java Chapter20_Lesson1
//
// The transfer matrix is G(s) = C (sI - A)^(-1) B + D.
// A realization is minimal iff (A,B) is reachable and (C,A) is observable.

public class Chapter20_Lesson1 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] identity(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int k = A[0].length;
        int c = B[0].length;
        if (B.length != k) throw new IllegalArgumentException("Dimension mismatch.");
        double[][] M = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                for (int t = 0; t < k; t++) {
                    M[i][j] += A[i][t] * B[t][j];
                }
            }
        }
        return M;
    }

    static double[][] hstack(double[][][] blocks) {
        int r = blocks[0].length;
        int totalCols = 0;
        for (double[][] B : blocks) totalCols += B[0].length;
        double[][] M = zeros(r, totalCols);
        int col0 = 0;
        for (double[][] B : blocks) {
            for (int i = 0; i < r; i++) {
                for (int j = 0; j < B[0].length; j++) {
                    M[i][col0 + j] = B[i][j];
                }
            }
            col0 += B[0].length;
        }
        return M;
    }

    static double[][] vstack(double[][][] blocks) {
        int c = blocks[0][0].length;
        int totalRows = 0;
        for (double[][] B : blocks) totalRows += B.length;
        double[][] M = zeros(totalRows, c);
        int row0 = 0;
        for (double[][] B : blocks) {
            for (int i = 0; i < B.length; i++) {
                for (int j = 0; j < c; j++) {
                    M[row0 + i][j] = B[i][j];
                }
            }
            row0 += B.length;
        }
        return M;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++) System.arraycopy(input[i], 0, M[i], 0, cols);

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            }
            if (Math.abs(M[pivot][c]) <= tol) continue;

            double[] tmp = M[pivot];
            M[pivot] = M[r];
            M[r] = tmp;

            double piv = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= piv;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] reachabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(A, Ak);
        }
        return hstack(blocks);
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(C, Ak);
            Ak = multiply(Ak, A);
        }
        return vstack(blocks);
    }

    static boolean isReachable(double[][] A, double[][] B) {
        return rank(reachabilityMatrix(A, B), 1e-10) == A.length;
    }

    static boolean isObservable(double[][] A, double[][] C) {
        return rank(observabilityMatrix(A, C), 1e-10) == A.length;
    }

    static boolean isMinimal(double[][] A, double[][] B, double[][] C) {
        return isReachable(A, B) && isObservable(A, C);
    }

    static void report(String name, double[][] A, double[][] B, double[][] C) {
        System.out.println("\n" + name);
        System.out.println("-".repeat(name.length()));
        System.out.println("rank(R_n) = " + rank(reachabilityMatrix(A, B), 1e-10) + " of n = " + A.length);
        System.out.println("rank(O_n) = " + rank(observabilityMatrix(A, C), 1e-10) + " of n = " + A.length);
        System.out.println("minimal?  = " + (isMinimal(A, B, C) ? "yes" : "no"));
    }

    public static void main(String[] args) {
        double[][] A_nonmin = {{-1.0, 0.0},
                               { 0.0,-2.0}};
        double[][] B_nonmin = {{1.0},
                               {1.0}};
        double[][] C_nonmin = {{0.0, 1.0}};

        double[][] A_min = {{-2.0}};
        double[][] B_min = {{1.0}};
        double[][] C_min = {{1.0}};

        report("Two-state nonminimal realization", A_nonmin, B_nonmin, C_nonmin);
        report("One-state minimal realization", A_min, B_min, C_min);

        System.out.println("\nBoth realizations have external transfer behavior G(s)=1/(s+2),");
        System.out.println("but the first realization contains one unobservable internal mode.");
    }
}
