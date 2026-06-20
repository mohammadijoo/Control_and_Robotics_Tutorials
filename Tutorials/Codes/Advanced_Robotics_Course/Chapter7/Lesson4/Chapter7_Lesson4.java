import java.util.Arrays;
import java.util.Random;

public class RobustGraspJava {

    private static double[] randomUnitVector3(Random rng) {
        double x = rng.nextGaussian();
        double y = rng.nextGaussian();
        double z = rng.nextGaussian();
        double n = Math.sqrt(x * x + y * y + z * z);
        if (n < 1e-12) {
            return new double[]{1.0, 0.0, 0.0};
        }
        return new double[]{x / n, y / n, z / n};
    }

    private static double ferrariCannyApprox(double[][] W,
                                             int numDirs,
                                             Random rng) {
        int m = W.length;
        double qVal = Double.POSITIVE_INFINITY;
        for (int k = 0; k < numDirs; ++k) {
            double[] d = randomUnitVector3(rng);
            double qDir = Double.NEGATIVE_INFINITY;
            for (int j = 0; j < m; ++j) {
                double s = d[0] * W[j][0] + d[1] * W[j][1] + d[2] * W[j][2];
                if (s > qDir) {
                    qDir = s;
                }
            }
            if (qDir < qVal) {
                qVal = qDir;
            }
        }
        return qVal;
    }

    public static double[] robustQualityMC(double[][] W0,
                                           double sigmaW,
                                           int numSamples,
                                           double delta) {
        Random rng = new Random(42L);
        double[] qVals = new double[numSamples];

        for (int s = 0; s < numSamples; ++s) {
            double[][] W = new double[W0.length][3];
            for (int j = 0; j < W0.length; ++j) {
                for (int k = 0; k < 3; ++k) {
                    double noise = rng.nextGaussian() * sigmaW;
                    W[j][k] = W0[j][k] + noise;
                }
            }
            qVals[s] = ferrariCannyApprox(W, 64, rng);
        }

        // mean
        double mean = 0.0;
        for (double q : qVals) {
            mean += q;
        }
        mean /= numSamples;

        // delta-quantile
        Arrays.sort(qVals);
        int idx = (int) (delta * numSamples);
        if (idx < 0) idx = 0;
        if (idx >= numSamples) idx = numSamples - 1;
        double qDelta = qVals[idx];

        return new double[]{mean, qDelta};
    }

    public static void main(String[] args) {
        double[][] W0 = new double[][]{
            { 1.0,  0.8,  0.2},
            {-1.0,  0.8, -0.2},
            { 0.8, -0.8,  0.3},
            {-0.8, -0.8, -0.3}
        };

        double[] res = robustQualityMC(W0, 0.05, 500, 0.1);
        System.out.println("Expected quality = " + res[0]);
        System.out.println("0.1-quantile quality = " + res[1]);
    }
}
      
