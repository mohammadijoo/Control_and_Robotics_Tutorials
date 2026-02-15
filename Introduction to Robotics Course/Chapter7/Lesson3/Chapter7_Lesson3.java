import java.util.Arrays;

public class ToFCorrelation {
    public static void main(String[] args) {
        double fs = 200000.0;
        double f0 = 40000.0;
        double vAir = 343.0;

        int N = 2000;
        int pulseN = 200;

        double[] s = new double[N];
        double[] r = new double[N];

        // transmit pulse
        for (int n = 0; n < pulseN; n++) {
            double t = n / fs;
            s[n] = Math.sin(2.0 * Math.PI * f0 * t);
        }

        // delayed echo
        double Ttrue = 0.0075;
        int dSamp = (int)(Ttrue * fs);
        double alpha = 0.6;
        for (int n = dSamp; n < N; n++) {
            r[n] = alpha * s[n - dSamp];
        }

        // full correlation
        double[] C = new double[2 * N - 1];
        int offset = N - 1;
        for (int k = -(N - 1); k <= (N - 1); k++) {
            double sum = 0.0;
            for (int n = 0; n < N; n++) {
                int idx = n - k;
                if (idx >= 0 && idx < N) sum += r[n] * s[idx];
            }
            C[k + offset] = sum;
        }

        int kHat = 0;
        double maxVal = C[0];
        for (int i = 1; i < C.length; i++) {
            if (C[i] > maxVal) {
                maxVal = C[i];
                kHat = i - offset;
            }
        }

        double THat = kHat / fs;
        double dHat = vAir * THat / 2.0;

        System.out.println("Estimated delay (s): " + THat);
        System.out.println("Estimated range (m): " + dHat);
    }
}
