// Chapter17_Lesson2.java
// CCF-OCF duality using plain Java arrays.
// Compile: javac Chapter17_Lesson2.java
// Run:     java Chapter17_Lesson2

public class Chapter17_Lesson2 {
    static double[][] zeros(int r, int c) { return new double[r][c]; }

    static double[][] eye(int n) {
        double[][] I = zeros(n, n);
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] transpose(double[][] A) {
        double[][] T = zeros(A[0].length, A.length);
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = zeros(A.length, B[0].length);
        for (int i = 0; i < A.length; i++)
            for (int k = 0; k < A[0].length; k++)
                for (int j = 0; j < B[0].length; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] ctrb(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Q = zeros(n, n);
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            double[][] col = multiply(Ak, B);
            for (int i = 0; i < n; i++) Q[i][k] = col[i][0];
            Ak = multiply(Ak, A);
        }
        return Q;
    }

    static double[][] obsv(double[][] A, double[][] C) {
        int n = A.length;
        double[][] O = zeros(n, n);
        double[][] Ak = eye(n);
        for (int k = 0; k < n; k++) {
            double[][] row = multiply(C, Ak);
            for (int j = 0; j < n; j++) O[k][j] = row[0][j];
            Ak = multiply(Ak, A);
        }
        return O;
    }

    static int rank(double[][] input) {
        double[][] A = copy(input);
        int m = A.length, n = A[0].length, r = 0;
        for (int col = 0; col < n && r < m; col++) {
            int p = r;
            for (int i = r+1; i < m; i++)
                if (Math.abs(A[i][col]) > Math.abs(A[p][col])) p = i;
            if (Math.abs(A[p][col]) < 1e-10) continue;
            double[] tmp = A[r]; A[r] = A[p]; A[p] = tmp;
            double pivot = A[r][col];
            for (int j = col; j < n; j++) A[r][j] /= pivot;
            for (int i = 0; i < m; i++) {
                if (i == r) continue;
                double f = A[i][col];
                for (int j = col; j < n; j++) A[i][j] -= f * A[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] copy(double[][] A) {
        double[][] B = zeros(A.length, A[0].length);
        for (int i = 0; i < A.length; i++)
            System.arraycopy(A[i], 0, B[i], 0, A[0].length);
        return B;
    }

    static double[] solve(double[][] A0, double[] b0) {
        double[][] A = copy(A0);
        double[] b = b0.clone();
        int n = b.length;
        for (int k = 0; k < n; k++) {
            int p = k;
            for (int i = k+1; i < n; i++)
                if (Math.abs(A[i][k]) > Math.abs(A[p][k])) p = i;
            double[] tr = A[k]; A[k] = A[p]; A[p] = tr;
            double tb = b[k]; b[k] = b[p]; b[p] = tb;
            double pivot = A[k][k];
            for (int j = k; j < n; j++) A[k][j] /= pivot;
            b[k] /= pivot;
            for (int i = 0; i < n; i++) {
                if (i == k) continue;
                double f = A[i][k];
                for (int j = k; j < n; j++) A[i][j] -= f*A[k][j];
                b[i] -= f*b[k];
            }
        }
        return b;
    }

    static double H(double[][] A, double[][] B, double[][] C, double D, double s) {
        int n = A.length;
        double[][] M = zeros(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                M[i][j] = (i == j ? s : 0.0) - A[i][j];
        double[] b = new double[n];
        for (int i = 0; i < n; i++) b[i] = B[i][0];
        double[] x = solve(M, b);
        double y = D;
        for (int i = 0; i < n; i++) y += C[0][i] * x[i];
        return y;
    }

    public static void main(String[] args) {
        int n = 3;
        double[] a = {4.0, 6.0, 4.0}; // [a0,a1,a2]
        double[] b = {3.0, 5.0, 2.0}; // [b0,b1,b2]

        double[][] Ac = zeros(n, n);
        for (int i = 0; i < n-1; i++) Ac[i][i+1] = 1.0;
        for (int j = 0; j < n; j++) Ac[n-1][j] = -a[j];

        double[][] Bc = zeros(n, 1);
        Bc[n-1][0] = 1.0;

        double[][] Cc = zeros(1, n);
        for (int j = 0; j < n; j++) Cc[0][j] = b[j];

        double[][] Ao = transpose(Ac);
        double[][] Bo = transpose(Cc);
        double[][] Co = transpose(Bc);

        System.out.println("rank ctrb(Ac,Bc) = " + rank(ctrb(Ac, Bc)));
        System.out.println("rank obsv(Ao,Co) = " + rank(obsv(Ao, Co)));

        for (double s : new double[]{0.5, 1.0, 2.0, 3.0}) {
            double hc = H(Ac, Bc, Cc, 0.0, s);
            double ho = H(Ao, Bo, Co, 0.0, s);
            System.out.printf("s=%.1f H_CCF=%.10f H_OCF=%.10f error=%.2e%n",
                              s, hc, ho, Math.abs(hc-ho));
        }
    }
}
