// Chapter26_Lesson2.java
// Modern Control - State augmentation with integral of tracking error
// Self-contained SISO augmented controllability and Ackermann example.
// Compile: javac Chapter26_Lesson2.java
// Run:     java Chapter26_Lesson2

public class Chapter26_Lesson2 {
    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] add(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = s * A[i][j];
        return C;
    }

    static double[][] mul(double[][] A, double[][] B) {
        int n = A.length, p = B.length, m = B[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                for (int k = 0; k < p; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[] matVec(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < x.length; j++)
                y[i] += A[i][j] * x[j];
        return y;
    }

    static double det3(double[][] M) {
        return M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
             - M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
             + M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);
    }

    static double[][] inverse3(double[][] M) {
        double d = det3(M);
        if (Math.abs(d) < 1e-12) throw new IllegalArgumentException("Singular matrix");
        double[][] inv = new double[3][3];
        inv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) / d;
        inv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) / d;
        inv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) / d;
        inv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) / d;
        inv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) / d;
        inv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) / d;
        inv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) / d;
        inv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) / d;
        inv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) / d;
        return inv;
    }

    static double[] rowTimesMat(double[] r, double[][] M) {
        double[] y = new double[M[0].length];
        for (int j = 0; j < M[0].length; j++)
            for (int k = 0; k < r.length; k++)
                y[j] += r[k] * M[k][j];
        return y;
    }

    static void printVector(String name, double[] v) {
        System.out.print(name + " = [ ");
        for (double x : v) System.out.printf("%10.6f ", x);
        System.out.println("]");
    }

    public static void main(String[] args) {
        double[][] Aa = {
            { 0.0,  1.0, 0.0},
            {-2.0, -3.0, 0.0},
            {-1.0,  0.0, 0.0}
        };
        double[] Ba = {0.0, 1.0, 0.0};

        double[] c1 = Ba;
        double[] c2 = matVec(Aa, Ba);
        double[] c3 = matVec(Aa, c2);
        double[][] Cc = {
            {c1[0], c2[0], c3[0]},
            {c1[1], c2[1], c3[1]},
            {c1[2], c2[2], c3[2]}
        };
        System.out.printf("det controllability(Aa,Ba) = %.6f%n", det3(Cc));

        // Desired poles: -2, -3, -4 -> phi(s)=s^3 + 9s^2 + 26s + 24.
        double[][] Aa2 = mul(Aa, Aa);
        double[][] Aa3 = mul(Aa2, Aa);
        double[][] phiAa = add(add(Aa3, scale(Aa2, 9.0)), add(scale(Aa, 26.0), scale(eye(3), 24.0)));

        double[] en = {0.0, 0.0, 1.0};
        double[][] invCc = inverse3(Cc);
        double[] temp = rowTimesMat(en, invCc);
        double[] Kaug = rowTimesMat(temp, phiAa);
        printVector("Kaug for u = -Kaug [x1 x2 q]^T", Kaug);
        System.out.printf("Equivalent Ki in u = -Kx*x + Ki*q is Ki = %.6f%n", -Kaug[2]);
    }
}
