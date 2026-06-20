// Chapter17_Lesson2.java
// Autocorrelation and PSD (compact Java example)

import java.util.Random;

public class Chapter17_Lesson2 {
    static double[] autocorrBiased(double[] x, int M) {
        int N = x.length;
        double mu = 0.0;
        for (double v : x) mu += v;
        mu /= N;

        double[] xc = new double[N];
        for (int i = 0; i < N; i++) xc[i] = x[i] - mu;

        double[] R = new double[M + 1];
        for (int m = 0; m <= M; m++) {
            for (int n = 0; n < N - m; n++) R[m] += xc[n] * xc[n + m];
            R[m] /= N;
        }
        return R;
    }

    static double[] evenExtend(double[] R) {
        int M = R.length - 1;
        double[] e = new double[2 * M];
        for (int i = 0; i <= M; i++) e[i] = R[i];
        for (int i = M - 1; i >= 1; i--) e[2 * M - i] = R[i];
        return e;
    }

    static double[] dftReal(double[] x) {
        int N = x.length;
        double[] S = new double[N];
        for (int k = 0; k < N; k++) {
            double re = 0.0, im = 0.0;
            for (int n = 0; n < N; n++) {
                double a = -2.0 * Math.PI * k * n / N;
                re += x[n] * Math.cos(a);
                im += x[n] * Math.sin(a);
            }
            S[k] = re; // autocorrelation extension is even, so imaginary part is near zero
        }
        return S;
    }

    public static void main(String[] args) {
        int N = 512, M = 20;
        double sigma = 1.2, b = 0.6;
        Random rng = new Random(17);

        double[] w = new double[N];
        for (int n = 0; n < N; n++) w[n] = sigma * rng.nextGaussian();

        double[] x = new double[N];
        x[0] = w[0];
        for (int n = 1; n < N; n++) x[n] = w[n] + b * w[n - 1];

        double[] Rw = autocorrBiased(w, M);
        double[] Rx = autocorrBiased(x, M);
        double[] Sx = dftReal(evenExtend(Rx));

        System.out.println("R_w[0] = " + Rw[0]);
        System.out.println("R_x[0] = " + Rx[0]);
        System.out.println("Theo R_x[0] = " + (sigma * sigma * (1 + b * b)));
        for (int k = 0; k < 8; k++) {
            System.out.println("PSD bin " + k + " = " + Sx[k]);
        }
    }
}
