import java.util.Random;

public class MonteCarloValidation {
    public static void main(String[] args) {
        double dt = 0.05;
        double[][] A = { {1.0, dt}, {0.0, 1.0} };
        double[] B = {0.5 * dt * dt, dt};
        double[] K = {-2.0, -1.0};

        double pMax = 5.0;
        double vMax = 3.0;
        int T = 200;
        int N_trials = 5000;
        double sigmaW = 0.05;

        Random rng = new Random(42L);

        int failures = 0;
        for (int i = 0; i < N_trials; ++i) {
            if (simulateTrial(A, B, K, dt, pMax, vMax, T, sigmaW, rng) == 1) {
                failures++;
            }
        }
        double pHat = (double) failures / N_trials;
        System.out.printf("Empirical failure rate: %.4f%n", pHat);
    }

    static int simulateTrial(double[][] A, double[] B, double[] K,
                             double dt, double pMax, double vMax,
                             int T, double sigmaW, Random rng) {
        double p0 = -4.0 + 8.0 * rng.nextDouble();
        double v0 = -2.0 + 4.0 * rng.nextDouble();
        double[] x = {p0, v0};
        for (int k = 0; k < T; ++k) {
            double u = K[0] * x[0] + K[1] * x[1];
            double w0 = sigmaW * rng.nextGaussian();
            double w1 = sigmaW * rng.nextGaussian();
            double pNext = A[0][0] * x[0] + A[0][1] * x[1] + B[0] * u + w0;
            double vNext = A[1][0] * x[0] + A[1][1] * x[1] + B[1] * u + w1;
            x[0] = pNext;
            x[1] = vNext;
            if (Math.abs(x[0]) > pMax || Math.abs(x[1]) > vMax) {
                return 1;
            }
        }
        return 0;
    }
}
      
