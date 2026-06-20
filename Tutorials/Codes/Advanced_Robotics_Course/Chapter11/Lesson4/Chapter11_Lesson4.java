public class SkillLibrary {
    private double[][] centers; // M x d

    public SkillLibrary(int numSkills, int dim) {
        centers = new double[numSkills][dim];
    }

    public int[] kMeans(double[][] segFeatures, int numIters) {
        int K = centers.length;
        int N = segFeatures.length;
        int d = segFeatures[0].length;
        int[] z = new int[N];

        // Initialize centers (first K segments)
        for (int k = 0; k < K; ++k) {
            System.arraycopy(segFeatures[k], 0, centers[k], 0, d);
        }

        for (int it = 0; it < numIters; ++it) {
            // Assignment step
            for (int n = 0; n < N; ++n) {
                z[n] = argMinDist(segFeatures[n]);
            }
            // Update step
            double[][] newCenters = new double[K][d];
            int[] counts = new int[K];
            for (int n = 0; n < N; ++n) {
                int k = z[n];
                counts[k]++;
                for (int j = 0; j < d; ++j) {
                    newCenters[k][j] += segFeatures[n][j];
                }
            }
            for (int k = 0; k < K; ++k) {
                if (counts[k] == 0) continue;
                for (int j = 0; j < d; ++j) {
                    newCenters[k][j] /= counts[k];
                }
                centers[k] = newCenters[k];
            }
        }
        return z;
    }

    private int argMinDist(double[] x) {
        int bestK = 0;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int k = 0; k < centers.length; ++k) {
            double d2 = 0.0;
            for (int j = 0; j < x.length; ++j) {
                double diff = x[j] - centers[k][j];
                d2 += diff * diff;
            }
            if (d2 < bestDist) {
                bestDist = d2;
                bestK = k;
            }
        }
        return bestK;
    }

    public double[][] getCenters() {
        return centers;
    }
}
      
