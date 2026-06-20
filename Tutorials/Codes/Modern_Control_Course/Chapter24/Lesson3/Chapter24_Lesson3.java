// Chapter24_Lesson3.java
// State transformations for MIMO pole placement without external libraries.
// Compile: javac Chapter24_Lesson3.java
// Run:     java Chapter24_Lesson3

public class Chapter24_Lesson3 {
    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, n = B.length, c = B[0].length;
        double[][] C = new double[r][c];
        for (int i = 0; i < r; i++)
            for (int k = 0; k < n; k++)
                for (int j = 0; j < c; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] subtract(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2*n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = A[i][j];
            aug[i][n+i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            if (Math.abs(aug[pivot][col]) < 1e-12) throw new RuntimeException("Singular matrix");
            double[] temp = aug[pivot]; aug[pivot] = aug[col]; aug[col] = temp;
            double div = aug[col][col];
            for (int j = 0; j < 2*n; j++) aug[col][j] /= div;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2*n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                inv[i][j] = aug[i][n+j];
        return inv;
    }

    static double frobeniusNorm(double[][] M) {
        double s = 0.0;
        for (double[] row : M)
            for (double v : row) s += v*v;
        return Math.sqrt(s);
    }

    static void printMatrix(String name, double[][] M) {
        System.out.println(name);
        for (double[] row : M) {
            for (double v : row) System.out.printf("%12.6f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] Abar = {
            {0,1,0,0},
            {0,0,0,0},
            {0,0,0,1},
            {0,0,0,0}
        };
        double[][] Bbar = {
            {0,0},
            {1,0},
            {0,0},
            {0,1}
        };
        double[][] T = {
            {1.0,0.2,0.0,0.1},
            {0.1,1.0,0.2,0.0},
            {0.0,0.1,1.0,0.3},
            {0.2,0.0,0.1,1.0}
        };
        double[][] F = {
            {6.0,5.0,0.0,0.0},
            {0.0,0.0,20.0,9.0}
        };

        double[][] Tinv = inverse(T);
        double[][] A = multiply(multiply(T, Abar), Tinv);
        double[][] B = multiply(T, Bbar);
        double[][] K = multiply(F, Tinv);
        double[][] Acl = subtract(A, multiply(B, K));
        double[][] Abarcl = subtract(Abar, multiply(Bbar, F));
        double[][] check = subtract(multiply(multiply(Tinv, Acl), T), Abarcl);

        printMatrix("K = F*T^{-1}:", K);
        printMatrix("A - B*K:", Acl);
        System.out.println("Similarity residual Frobenius norm = " + frobeniusNorm(check));
        System.out.println("Expected poles from the transformed design: -2, -3, -4, -5");
    }
}
