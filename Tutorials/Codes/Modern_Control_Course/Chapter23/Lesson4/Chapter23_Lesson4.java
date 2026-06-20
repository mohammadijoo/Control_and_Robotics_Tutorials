// Chapter23_Lesson4.java
// Mapping time-domain specifications to desired poles and Ackermann pole placement.
// Compile: javac Chapter23_Lesson4.java && java Chapter23_Lesson4

import java.util.Arrays;

public class Chapter23_Lesson4 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, r = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int k = 0; k < r; k++)
                for (int j = 0; j < m; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[] multiply(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < x.length; j++) y[i] += A[i][j] * x[j];
        return y;
    }

    static double[][] add(double[][] A, double[][] B, double scaleB) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] + scaleB * B[i][j];
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        double[][] B = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++) B[i][j] = s * A[i][j];
        return B;
    }

    static double[][] power(double[][] A, int p) {
        double[][] R = eye(A.length), B = A;
        while (p > 0) {
            if ((p & 1) == 1) R = multiply(R, B);
            B = multiply(B, B);
            p >>= 1;
        }
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
            int piv = col;
            for (int r = col + 1; r < n; r++)
                if (Math.abs(aug[r][col]) > Math.abs(aug[piv][col])) piv = r;
            if (Math.abs(aug[piv][col]) < 1e-12) throw new RuntimeException("singular matrix");
            double[] tmp = aug[piv]; aug[piv] = aug[col]; aug[col] = tmp;
            double div = aug[col][col];
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= div;
            for (int r = 0; r < n; r++) if (r != col) {
                double f = aug[r][col];
                for (int j = 0; j < 2 * n; j++) aug[r][j] -= f * aug[col][j];
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) System.arraycopy(aug[i], n, inv[i], 0, n);
        return inv;
    }

    static double dampingRatioFromOvershoot(double percentOvershoot) {
        double m = percentOvershoot / 100.0;
        double L = Math.log(m);
        return -L / Math.sqrt(Math.PI * Math.PI + L * L);
    }

    static double[] secondOrderPolynomialFromSpecs(double percentOvershoot, double settlingTime) {
        double zeta = dampingRatioFromOvershoot(percentOvershoot);
        double wn = 4.0 / (zeta * settlingTime);
        return new double[]{1.0, 2.0 * zeta * wn, wn * wn};
    }

    static double[] convolve(double[] a, double[] b) {
        double[] c = new double[a.length + b.length - 1];
        for (int i = 0; i < a.length; i++)
            for (int j = 0; j < b.length; j++) c[i + j] += a[i] * b[j];
        return c;
    }

    static double[][] controllabilityMatrix(double[][] A, double[] b) {
        int n = A.length;
        double[][] W = new double[n][n];
        double[] col = Arrays.copyOf(b, n);
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < n; i++) W[i][k] = col[i];
            col = multiply(A, col);
        }
        return W;
    }

    static double[] ackermannGain(double[][] A, double[] b, double[] desiredPoly) {
        int n = A.length;
        double[][] W = controllabilityMatrix(A, b);
        double[][] Winv = inverse(W);
        double[][] phi = power(A, n);
        for (int i = 0; i < n; i++) {
            int pow = n - 1 - i;
            phi = add(phi, scale(power(A, pow), desiredPoly[i + 1]), 1.0);
        }
        double[][] temp = multiply(Winv, phi);
        return Arrays.copyOf(temp[n - 1], n);
    }

    public static void main(String[] args) {
        double[][] A = {{0, 1, 0}, {0, 0, 1}, {-2, -3, -1}};
        double[] b = {0, 0, 1};

        double Mp = 10.0, Ts = 2.0;
        double zeta = dampingRatioFromOvershoot(Mp);
        double wn = 4.0 / (zeta * Ts);
        double sigma = zeta * wn;

        double[] secondOrder = secondOrderPolynomialFromSpecs(Mp, Ts);
        double[] extraPole = {1.0, 6.0 * sigma}; // factor s + 6*sigma
        double[] desiredPoly = convolve(secondOrder, extraPole);
        double[] K = ackermannGain(A, b, desiredPoly);

        System.out.println("zeta = " + zeta);
        System.out.println("omega_n = " + wn);
        System.out.println("desired polynomial = " + Arrays.toString(desiredPoly));
        System.out.println("K = " + Arrays.toString(K));
    }
}
