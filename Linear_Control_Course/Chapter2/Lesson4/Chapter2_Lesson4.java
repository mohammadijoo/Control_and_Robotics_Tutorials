public class FirstOrderConvolution {
    public static void main(String[] args) {
        double dt = 0.001;
        int N = 5000;
        double tau = 0.2;

        double[] g = new double[N];
        double[] u = new double[N];
        double[] y = new double[N];

        for (int k = 0; k < N; ++k) {
            double t = k * dt;
            g[k] = (t >= 0.0) ? (1.0 / tau) * Math.exp(-t / tau) : 0.0;
            u[k] = (t >= 0.0) ? 1.0 : 0.0; // unit step
        }

        for (int k = 0; k < N; ++k) {
            double sum = 0.0;
            for (int j = 0; j <= k; ++j) {
                sum += g[j] * u[k - j] * dt;
            }
            y[k] = sum;
        }

        for (int k = 0; k < N; k += 500) {
            double t = k * dt;
            double analytic = 1.0 - Math.exp(-t / tau);
            System.out.printf("t=%.3f  y(t)~%.4f  analytic=%.4f%n", t, y[k], analytic);
        }
    }
}
