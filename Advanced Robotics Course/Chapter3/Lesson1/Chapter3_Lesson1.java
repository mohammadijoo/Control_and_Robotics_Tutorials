import java.util.Random;

public class PCMonteCarlo {
    private static final Random rng = new Random();

    public static boolean trialSuccess(int numSamples, int dim, double radius) {
        double[] center = new double[dim];
        for (int i = 0; i < dim; ++i) {
            center[i] = 0.5;
        }
        for (int i = 0; i < numSamples; ++i) {
            double dist2 = 0.0;
            for (int d = 0; d < dim; ++d) {
                double x = rng.nextDouble();
                double diff = x - center[d];
                dist2 += diff * diff;
            }
            if (Math.sqrt(dist2) <= radius) {
                return true;
            }
        }
        return false;
    }

    public static double estimateSuccessProb(int numSamples,
                                             int dim,
                                             double radius,
                                             int trials) {
        int successes = 0;
        for (int t = 0; t < trials; ++t) {
            if (trialSuccess(numSamples, dim, radius)) {
                successes++;
            }
        }
        return ((double) successes) / trials;
    }

    public static void main(String[] args) {
        int[] ns = {10, 50, 100, 200, 500, 1000};
        for (int n : ns) {
            double p = estimateSuccessProb(n, 4, 0.1, 1000);
            System.out.println("n = " + n +
                               ", approx P(success) = " + p);
        }
    }
}
      
