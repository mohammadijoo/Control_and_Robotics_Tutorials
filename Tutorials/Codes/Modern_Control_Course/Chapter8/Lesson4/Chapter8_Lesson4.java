// Chapter8_Lesson4.java
// Computing Phi(t) from a known Jordan form: Phi(t)=P exp(Jt) P^{-1}.
// Compile: javac Chapter8_Lesson4.java
// Run:     java Chapter8_Lesson4

public class Chapter8_Lesson4 {
    static double[][] zeros(int r, int c) {
        return new double[r][c];
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length;
        int m = A[0].length;
        int c = B[0].length;
        double[][] C = zeros(r, c);
        for (int i = 0; i < r; i++) {
            for (int k = 0; k < m; k++) {
                for (int j = 0; j < c; j++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double factorial(int n) {
        double f = 1.0;
        for (int i = 2; i <= n; i++) f *= i;
        return f;
    }

    static double[][] jordanBlockExp(double lambda, int size, double t) {
        double[][] B = zeros(size, size);
        double e = Math.exp(lambda * t);
        for (int i = 0; i < size; i++) {
            for (int j = i; j < size; j++) {
                int p = j - i;
                B[i][j] = e * Math.pow(t, p) / factorial(p);
            }
        }
        return B;
    }

    static double[][] blockDiag(double[][]... blocks) {
        int n = 0;
        for (double[][] B : blocks) n += B.length;
        double[][] M = zeros(n, n);
        int off = 0;
        for (double[][] B : blocks) {
            for (int i = 0; i < B.length; i++) {
                for (int j = 0; j < B.length; j++) {
                    M[off + i][off + j] = B[i][j];
                }
            }
            off += B.length;
        }
        return M;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = zeros(n, 2 * n);

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n + i] = 1.0;
        }

        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            }
            if (Math.abs(aug[pivot][col]) < 1e-12) {
                throw new RuntimeException("Singular matrix.");
            }

            double[] tmp = aug[col];
            aug[col] = aug[pivot];
            aug[pivot] = tmp;

            double p = aug[col][col];
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= p;

            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2 * n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }

        double[][] inv = zeros(n, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) inv[i][j] = aug[i][n + j];
        }
        return inv;
    }

    static void printMatrix(double[][] A, String name) {
        System.out.println(name + " =");
        for (double[] row : A) {
            for (double v : row) System.out.printf("%14.7f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] P = {
            {1.0, 1.0, 0.0},
            {0.0, 1.0, 1.0},
            {1.0, 0.0, 1.0}
        };

        double[][] expJ = blockDiag(
            jordanBlockExp(2.0, 2, 0.40),
            jordanBlockExp(-1.0, 1, 0.40)
        );

        double[][] Phi = multiply(multiply(P, expJ), inverse(P));
        printMatrix(Phi, "Phi(0.40)");
    }
}
