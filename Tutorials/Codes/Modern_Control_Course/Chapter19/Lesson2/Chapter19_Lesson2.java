// Chapter19_Lesson2.java
// Observable/unobservable subspaces using from-scratch RREF operations.
// Compile/run:
//   javac Chapter19_Lesson2.java
//   java Chapter19_Lesson2

public class Chapter19_Lesson2 {
    static final double EPS = 1e-10;

    static double[][] multiply(double[][] A, double[][] B) {
        int m = A.length, n = B[0].length, inner = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0.0;
                for (int k = 0; k < inner; k++) sum += A[i][k] * B[k][j];
                C[i][j] = sum;
            }
        }
        return C;
    }

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length, p = C.length;
        double[][] O = new double[p * n][n];
        double[][] Ak = eye(n);
        for (int block = 0; block < n; block++) {
            double[][] CAk = multiply(C, Ak);
            for (int i = 0; i < p; i++) {
                System.arraycopy(CAk[i], 0, O[block * p + i], 0, n);
            }
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static double[][] rref(double[][] M) {
        int rows = M.length, cols = M[0].length;
        double[][] R = new double[rows][cols];
        for (int i = 0; i < rows; i++) System.arraycopy(M[i], 0, R[i], 0, cols);

        int lead = 0;
        for (int r = 0; r < rows && lead < cols; r++) {
            int i = r;
            while (i < rows && Math.abs(R[i][lead]) < EPS) i++;
            if (i == rows) { lead++; r--; continue; }
            double[] temp = R[r]; R[r] = R[i]; R[i] = temp;
            double pivot = R[r][lead];
            for (int j = 0; j < cols; j++) R[r][j] /= pivot;
            for (int rr = 0; rr < rows; rr++) {
                if (rr == r) continue;
                double factor = R[rr][lead];
                for (int j = 0; j < cols; j++) R[rr][j] -= factor * R[r][j];
            }
            lead++;
        }
        return R;
    }

    static int rank(double[][] M) {
        double[][] R = rref(M);
        int rank = 0;
        for (double[] row : R) {
            boolean nonzero = false;
            for (double v : row) if (Math.abs(v) > EPS) { nonzero = true; break; }
            if (nonzero) rank++;
        }
        return rank;
    }

    static double[][] nullspace(double[][] M) {
        double[][] R = rref(M);
        int rows = R.length, cols = R[0].length;
        boolean[] pivot = new boolean[cols];
        int[] pivotColForRow = new int[rows];
        for (int i = 0; i < rows; i++) pivotColForRow[i] = -1;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (Math.abs(R[i][j] - 1.0) < EPS) {
                    boolean isPivot = true;
                    for (int k = 0; k < rows; k++) {
                        if (k != i && Math.abs(R[k][j]) > EPS) { isPivot = false; break; }
                    }
                    if (isPivot) { pivot[j] = true; pivotColForRow[i] = j; break; }
                }
            }
        }

        int freeCount = 0;
        for (boolean b : pivot) if (!b) freeCount++;
        double[][] N = new double[cols][freeCount];
        int colIndex = 0;
        for (int free = 0; free < cols; free++) {
            if (pivot[free]) continue;
            N[free][colIndex] = 1.0;
            for (int i = 0; i < rows; i++) {
                int pc = pivotColForRow[i];
                if (pc >= 0) N[pc][colIndex] = -R[i][free];
            }
            colIndex++;
        }
        return N;
    }

    static double frobeniusNorm(double[][] M) {
        double s = 0.0;
        for (double[] row : M) for (double v : row) s += v * v;
        return Math.sqrt(s);
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name + ":");
        for (double[] row : M) {
            for (double v : row) System.out.printf("%10.5f ", v);
            System.out.println();
        }
        System.out.println();
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0, 0.0, 0.0},
            {-2.0, -3.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 1.0},
            {0.0, 0.0, -5.0, -1.0}
        };
        double[][] C = {{1.0, 0.0, 0.0, 0.0}};

        double[][] O = observabilityMatrix(A, C);
        double[][] N = nullspace(O);
        double[][] OAN = multiply(O, multiply(A, N));

        printMatrix("Observability matrix O_n", O);
        System.out.println("rank(O_n) = " + rank(O) + " out of n = " + A.length + "\n");
        printMatrix("Basis for unobservable subspace ker(O_n)", N);
        System.out.printf("Invariance check ||O_n A N||_F = %.5e\n", frobeniusNorm(OAN));
    }
}
