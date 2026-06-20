import java.util.function.DoubleFunction;

public class ConvolutionForcedResponse {

    // Minimal matrix utilities (educational only)
    static double[][] matMul(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, p = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int k = 0; k < p; k++)
                for (int j = 0; j < m; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[] matVec(double[][] A, double[] x) {
        int n = A.length, m = x.length;
        double[] y = new double[n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                y[i] += A[i][j] * x[j];
        return y;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] matScale(double[][] A, double s) {
        int n = A.length, m = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                C[i][j] = s * A[i][j];
        return C;
    }

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    // Truncated series expm(A*t) ≈ Σ_{k=0}^{K} (A*t)^k / k!
    // Educational only; replace with Padé + scaling/squaring for robustness.
    static double[][] expmSeries(double[][] A, double t, int K) {
        int n = A.length;
        double[][] At = matScale(A, t);
        double[][] E = eye(n);
        double[][] term = eye(n);

        for (int k = 1; k <= K; k++) {
            term = matMul(term, At);
            double invFact = 1.0;
            for (int j = 2; j <= k; j++) invFact /= j;
            E = matAdd(E, matScale(term, invFact));
        }
        return E;
    }

    static double[] addVec(double[] a, double[] b) {
        double[] c = new double[a.length];
        for (int i = 0; i < a.length; i++) c[i] = a[i] + b[i];
        return c;
    }

    static double[] scaleVec(double[] a, double s) {
        double[] b = new double[a.length];
        for (int i = 0; i < a.length; i++) b[i] = s * a[i];
        return b;
    }

    public static void main(String[] args) {
        double[][] A = { {0.0, 1.0}, {-2.0, -3.0} };
        double[][] B = { {0.0}, {1.0} };
        double[] x0 = {1.0, 0.0};

        DoubleFunction<double[]> uFun = (double tt) -> {
            return new double[] { (tt >= 0.0 && tt <= 2.0) ? 1.0 : 0.0 };
        };

        double T = 6.0, dt = 0.01;
        int N = (int)Math.round(T/dt);

        double[][] x = new double[N+1][2];

        // Precompute expm(A * (k*dt)) as needed (simple, but expensive if done naively)
        for (int k = 0; k <= N; k++) {
            double tk = k * dt;

            // forced part via trapezoidal sum
            double[] acc = new double[] {0.0, 0.0};
            for (int j = 0; j < k; j++) {
                double sj  = j * dt;
                double sj1 = (j+1) * dt;

                double[] uj  = uFun.apply(sj);
                double[] uj1 = uFun.apply(sj1);

                double[][] K0 = matMul(expmSeries(A, tk - sj, 20), B);
                double[][] K1 = matMul(expmSeries(A, tk - sj1, 20), B);

                double[] v0 = matVec(K0, uj);
                double[] v1 = matVec(K1, uj1);
                acc = addVec(acc, scaleVec(addVec(v0, v1), 0.5 * dt));
            }

            double[] zi = matVec(expmSeries(A, tk, 20), x0);
            x[k] = addVec(zi, acc);
        }

        System.out.println("x(0) = [" + x[0][0] + ", " + x[0][1] + "]");
        System.out.println("x(6) = [" + x[N][0] + ", " + x[N][1] + "]");
    }
}
