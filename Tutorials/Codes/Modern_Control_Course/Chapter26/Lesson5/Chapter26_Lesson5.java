// Chapter26_Lesson5.java
// Servo design by state augmentation using only small from-scratch matrix routines.
// Compile/run: javac Chapter26_Lesson5.java && java Chapter26_Lesson5

import java.util.Arrays;

public class Chapter26_Lesson5 {
    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, q = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                for (int k = 0; k < q; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scale(double a, double[][] A) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                C[i][j] = a * A[i][j];
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] power(double[][] A, int k) {
        double[][] R = identity(A.length);
        for (int i = 0; i < k; i++) R = multiply(R, A);
        return R;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, aug[i], 0, n);
            aug[i][n + i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
            double[] tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;
            double div = aug[col][col];
            if (Math.abs(div) < 1e-12) throw new RuntimeException("Singular matrix");
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= div;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int j = 0; j < 2 * n; j++) aug[r][j] -= factor * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++)
            System.arraycopy(aug[i], n, inv[i], 0, n);
        return inv;
    }

    static double[] polynomialFromRoots(double[] roots) {
        double[] c = {1.0};
        for (double r : roots) {
            double[] next = new double[c.length + 1];
            for (int i = 0; i < c.length; i++) {
                next[i] += c[i];
                next[i + 1] += -r * c[i];
            }
            c = next;
        }
        return c;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] W = new double[n][n];
        for (int k = 0; k < n; k++) {
            double[][] block = multiply(power(A, k), B);
            for (int i = 0; i < n; i++) W[i][k] = block[i][0];
        }
        return W;
    }

    static double[][] matrixPolynomial(double[][] A, double[] coeff) {
        int n = A.length;
        double[][] phi = new double[n][n];
        for (int i = 0; i <= n; i++) {
            int pow = n - i;
            phi = add(phi, scale(coeff[i], power(A, pow)));
        }
        return phi;
    }

    static double[] rowTimesMatrix(double[] row, double[][] A) {
        double[] y = new double[A[0].length];
        for (int j = 0; j < A[0].length; j++)
            for (int k = 0; k < row.length; k++) y[j] += row[k] * A[k][j];
        return y;
    }

    static double[] matVec(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < x.length; j++) y[i] += A[i][j] * x[j];
        return y;
    }

    public static void main(String[] args) {
        double[][] Aa = {
            {0.0, 1.0, 0.0},
            {-2.0, -0.8, 0.0},
            {-1.0, 0.0, 0.0}
        };
        double[][] Ba = {{0.0}, {1.0}, {0.0}};
        double[][] Ea = {{0.0}, {0.0}, {1.0}};
        double[][] C = {{1.0, 0.0}};

        double[] roots = {-2.0, -2.5, -3.0};
        double[] coeff = polynomialFromRoots(roots);
        double[][] Wc = controllabilityMatrix(Aa, Ba);
        double[][] phiA = matrixPolynomial(Aa, coeff);

        double[] enT = {0.0, 0.0, 1.0};
        double[] Kaug = rowTimesMatrix(rowTimesMatrix(enT, inverse(Wc)), phiA);
        double[] Kx = {Kaug[0], Kaug[1]};
        double Ki = -Kaug[2];

        System.out.println("Kaug = " + Arrays.toString(Kaug));
        System.out.println("Kx = " + Arrays.toString(Kx));
        System.out.println("Ki = " + Ki);

        double[][] Acl = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Acl[i][j] = Aa[i][j] - Ba[i][0] * Kaug[j];

        double[] z = {0.0, 0.0, 0.0};
        double r = 1.0, dt = 0.001, tf = 8.0;
        for (int k = 0; k < (int)(tf / dt); k++) {
            double[] zdot = matVec(Acl, z);
            for (int i = 0; i < 3; i++) zdot[i] += Ea[i][0] * r;
            for (int i = 0; i < 3; i++) z[i] += dt * zdot[i];
        }
        double y = C[0][0] * z[0] + C[0][1] * z[1];
        double u = -(Kx[0] * z[0] + Kx[1] * z[1]) + Ki * z[2];
        System.out.println("Final y = " + y);
        System.out.println("Final tracking error = " + (r - y));
        System.out.println("Final u = " + u);
    }
}
