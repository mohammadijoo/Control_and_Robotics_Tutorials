/*
Chapter18_Lesson5.java
Interpretation of Jordan form in control applications.

No external libraries are required. The example uses small dense matrices and
basic Gaussian elimination for numerical rank.

Compile and run:
    javac Chapter18_Lesson5.java
    java Chapter18_Lesson5
*/

public class Chapter18_Lesson5 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] jordanBlock(double lambda, int n) {
        double[][] J = eye(n);
        for (int i = 0; i < n; i++) J[i][i] = lambda;
        for (int i = 0; i < n - 1; i++) J[i][i + 1] = 1.0;
        return J;
    }

    static double factorial(int k) {
        double f = 1.0;
        for (int i = 2; i <= k; i++) f *= i;
        return f;
    }

    static double[][] jordanBlockExponential(double lambda, int n, double t) {
        double[][] E = new double[n][n];
        double scale = Math.exp(lambda * t);
        for (int i = 0; i < n; i++) {
            for (int j = i; j < n; j++) {
                int power = j - i;
                E[i][j] = scale * Math.pow(t, power) / factorial(power);
            }
        }
        return E;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int c = B[0].length;
        int inner = B.length;
        double[][] M = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                double s = 0.0;
                for (int k = 0; k < inner; k++) s += A[i][k] * B[k][j];
                M[i][j] = s;
            }
        }
        return M;
    }

    static double[][] hstack(double[][][] blocks) {
        int rows = blocks[0].length;
        int totalCols = 0;
        for (double[][] block : blocks) totalCols += block[0].length;
        double[][] M = new double[rows][totalCols];
        int col0 = 0;
        for (double[][] block : blocks) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < block[0].length; j++) M[i][col0 + j] = block[i][j];
            }
            col0 += block[0].length;
        }
        return M;
    }

    static double[][] vstack(double[][][] blocks) {
        int cols = blocks[0][0].length;
        int totalRows = 0;
        for (double[][] block : blocks) totalRows += block.length;
        double[][] M = new double[totalRows][cols];
        int row0 = 0;
        for (double[][] block : blocks) {
            for (int i = 0; i < block.length; i++) {
                for (int j = 0; j < cols; j++) M[row0 + i][j] = block[i][j];
            }
            row0 += block.length;
        }
        return M;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(A, Ak);
        }
        return hstack(blocks);
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(C, Ak);
            Ak = multiply(Ak, A);
        }
        return vstack(blocks);
    }

    static int rank(double[][] input) {
        double[][] M = new double[input.length][input[0].length];
        for (int i = 0; i < input.length; i++) System.arraycopy(input[i], 0, M[i], 0, input[0].length);
        int rows = M.length;
        int cols = M[0].length;
        int r = 0;
        double tol = 1e-10;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++) {
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            }
            if (Math.abs(M[pivot][c]) <= tol) continue;
            double[] tmp = M[r];
            M[r] = M[pivot];
            M[pivot] = tmp;
            double div = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= div;
            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double v : row) System.out.printf("%12.6f", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = jordanBlock(-1.0, 3);
        double[][] E = jordanBlockExponential(-1.0, 3, 2.0);
        printMatrix("A", A);
        printMatrix("exp(A*2) closed form", E);

        double[][] Bgood = {{0.0}, {0.0}, {1.0}};
        double[][] Bbad = {{1.0}, {0.0}, {0.0}};
        double[][] WcGood = controllabilityMatrix(A, Bgood);
        double[][] WcBad = controllabilityMatrix(A, Bbad);
        printMatrix("Ctrb(A,Bgood)", WcGood);
        System.out.println("rank = " + rank(WcGood));
        printMatrix("Ctrb(A,Bbad)", WcBad);
        System.out.println("rank = " + rank(WcBad));

        double[][] Cgood = {{1.0, 0.0, 0.0}};
        double[][] Cbad = {{0.0, 0.0, 1.0}};
        double[][] WoGood = observabilityMatrix(A, Cgood);
        double[][] WoBad = observabilityMatrix(A, Cbad);
        printMatrix("Obsv(A,Cgood)", WoGood);
        System.out.println("rank = " + rank(WoGood));
        printMatrix("Obsv(A,Cbad)", WoBad);
        System.out.println("rank = " + rank(WoBad));
    }
}
