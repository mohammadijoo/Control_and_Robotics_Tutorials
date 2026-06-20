// Chapter14_Lesson3.java
// Duality Between Controllability and Observability
// Compile: javac Chapter14_Lesson3.java
// Run:     java Chapter14_Lesson3

public class Chapter14_Lesson3 {
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
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int t = 0; t < k; t++)
                    M[i][j] += A[i][t] * B[t][j];
        return M;
    }

    static double[][] transpose(double[][] A) {
        int r = A.length;
        int c = A[0].length;
        double[][] T = zeros(c, r);
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] hstack(double[][][] blocks) {
        int rows = blocks[0].length;
        int totalCols = 0;
        for (double[][] B : blocks) totalCols += B[0].length;
        double[][] H = zeros(rows, totalCols);
        int col0 = 0;
        for (double[][] B : blocks) {
            int cols = B[0].length;
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    H[i][col0 + j] = B[i][j];
            col0 += cols;
        }
        return H;
    }

    static double[][] vstack(double[][][] blocks) {
        int cols = blocks[0][0].length;
        int totalRows = 0;
        for (double[][] B : blocks) totalRows += B.length;
        double[][] V = zeros(totalRows, cols);
        int row0 = 0;
        for (double[][] B : blocks) {
            int rows = B.length;
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    V[row0 + i][j] = B[i][j];
            row0 += rows;
        }
        return V;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            System.arraycopy(input[i], 0, M[i], 0, cols);

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++)
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;

            if (Math.abs(M[pivot][c]) <= tol) continue;

            double[] temp = M[pivot];
            M[pivot] = M[r];
            M[r] = temp;

            double div = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= div;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++)
                    M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] Ak = identity(n);
        for (int k = 0; k < n; k++) {
            blocks[k] = multiply(Ak, B);
            Ak = multiply(Ak, A);
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

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + " =");
        for (double[] row : M) {
            for (double x : row) System.out.printf("%10.4f ", x);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {-2.0, -3.0, -4.0}
        };
        double[][] B = {{0.0}, {0.0}, {1.0}};
        double[][] C = {{1.0, 0.0, 0.0}};

        int n = A.length;

        double[][] Ctrb = controllabilityMatrix(A, B);
        double[][] Obsv = observabilityMatrix(A, C);
        double[][] CtrbDual = controllabilityMatrix(transpose(A), transpose(C));
        double[][] ObsvDual = observabilityMatrix(transpose(A), transpose(B));

        printMatrix("Ctrb(A,B)", Ctrb);
        printMatrix("Obsv(A,C)", Obsv);

        System.out.println("\nrank Ctrb(A,B) = " + rank(Ctrb, 1e-10) + " of " + n);
        System.out.println("rank Obsv(A,C) = " + rank(Obsv, 1e-10) + " of " + n);
        System.out.println("rank Ctrb(A^T,C^T) = " + rank(CtrbDual, 1e-10) + " of " + n);
        System.out.println("rank Obsv(A^T,B^T) = " + rank(ObsvDual, 1e-10) + " of " + n);
    }
}
