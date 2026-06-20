// Chapter17_Lesson5.java
// Practical Considerations in Choosing Canonical Forms
// Compile: javac Chapter17_Lesson5.java
// Run:     java Chapter17_Lesson5

public class Chapter17_Lesson5 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] matmul(double[][] A, double[][] B) {
        int r = A.length, k = B.length, c = B[0].length;
        double[][] M = new double[r][c];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int t = 0; t < k; t++)
                    M[i][j] += A[i][t] * B[t][j];
        return M;
    }

    static double[][] transpose(double[][] A) {
        int r = A.length, c = A[0].length;
        double[][] T = new double[c][r];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] Aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) Aug[i][j] = A[i][j];
            Aug[i][n + i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int i = col + 1; i < n; i++)
                if (Math.abs(Aug[i][col]) > Math.abs(Aug[pivot][col])) pivot = i;
            if (Math.abs(Aug[pivot][col]) < 1e-12) throw new RuntimeException("Singular matrix");
            double[] temp = Aug[pivot]; Aug[pivot] = Aug[col]; Aug[col] = temp;
            double div = Aug[col][col];
            for (int j = 0; j < 2 * n; j++) Aug[col][j] /= div;
            for (int i = 0; i < n; i++) {
                if (i == col) continue;
                double factor = Aug[i][col];
                for (int j = 0; j < 2 * n; j++) Aug[i][j] -= factor * Aug[col][j];
            }
        }
        double[][] Inv = new double[n][n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Inv[i][j] = Aug[i][n + j];
        return Inv;
    }

    static double frobeniusNorm(double[][] A) {
        double s = 0.0;
        for (double[] row : A)
            for (double v : row) s += v * v;
        return Math.sqrt(s);
    }

    static double frobeniusCondition(double[][] A) {
        return frobeniusNorm(A) * frobeniusNorm(inverse(A));
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][] O = new double[n][n];
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            double[][] row = matmul(C, Ak);
            for (int j = 0; j < n; j++) O[k][j] = row[0][j];
            Ak = matmul(Ak, A);
        }
        return O;
    }

    static double[][] diag(double[] d) {
        int n = d.length;
        double[][] D = new double[n][n];
        for (int i = 0; i < n; i++) D[i][i] = d[i];
        return D;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name);
        for (double[] row : A) {
            for (double v : row) System.out.printf("%12.6f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A0 = {{0, 1, 0}, {0, 0, 1}, {-6, -11, -6}};
        double[][] C0 = {{1, 0, 0}};
        double[][] S = diag(new double[]{1.0, 0.05, 20.0});
        double[][] A = matmul(matmul(S, A0), inverse(S));
        double[][] C = matmul(C0, inverse(S));

        double[][] Ao = transpose(A0);
        double[][] Co = {{0, 0, 1}};
        double[][] O = observabilityMatrix(A, C);
        double[][] Oo = observabilityMatrix(Ao, Co);
        double[][] T_ocf = matmul(inverse(O), Oo);
        double[][] A_ocf = matmul(matmul(inverse(T_ocf), A), T_ocf);
        double[][] C_ocf = matmul(C, T_ocf);

        double[] lam = {-1.0, -2.0, -3.0};
        double[][] V = new double[3][3];
        for (int j = 0; j < 3; j++) {
            double l = lam[j];
            double[][] v0 = {{1.0}, {l}, {l * l}};
            double[][] v = matmul(S, v0);
            for (int i = 0; i < 3; i++) V[i][j] = v[i][0];
        }
        double[][] A_modal = matmul(matmul(inverse(V), A), V);

        printMatrix("Physical/scaled A:", A);
        printMatrix("Observable canonical A:", A_ocf);
        printMatrix("Observable canonical C:", C_ocf);
        System.out.printf("OCF transform condition proxy = %.6f%n%n", frobeniusCondition(T_ocf));
        printMatrix("Modal A:", A_modal);
        System.out.printf("Modal eigenvector condition proxy = %.6f%n", frobeniusCondition(V));

        double kModal = frobeniusCondition(V);
        double kOcf = frobeniusCondition(T_ocf);
        if (Math.min(kModal, kOcf) > 5000.0)
            System.out.println("Recommendation: both canonical transforms are poorly conditioned; keep physical/scaled coordinates or use an orthogonal Schur form.");
        else if (kModal < kOcf)
            System.out.println("Recommendation: modal coordinates are cleaner for this example.");
        else
            System.out.println("Recommendation: OCF is acceptable for observer algebra in this example.");
    }
}
