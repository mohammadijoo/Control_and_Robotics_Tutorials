import java.util.Random;

public class Stomp2D {

    private final Random rng = new Random(0L);

    private double obstacleCost(double[] x) {
        double d0 = 0.5;
        double d = Math.sqrt(x[0]*x[0] + x[1]*x[1]);
        if (d < d0) {
            double diff = d0 - d;
            return 0.5 * diff * diff;
        }
        return 0.0;
    }

    public double[][] stompStep(double[][] q, int numRollouts,
                                double noiseStd, double lambda, double eta) {
        int N = q.length;
        int dim = 2;

        double[][] eps = new double[numRollouts][N * dim];
        double[] costs = new double[numRollouts];

        for (int m = 0; m < numRollouts; ++m) {
            // White noise
            double[] e = new double[N * dim];
            for (int i = 0; i < N * dim; ++i) {
                e[i] = rng.nextGaussian() * noiseStd;
            }
            eps[m] = e;

            // Perturbed trajectory
            double[][] qPert = new double[N][dim];
            for (int k = 0; k < N; ++k) {
                qPert[k][0] = q[k][0] + e[2*k];
                qPert[k][1] = q[k][1] + e[2*k + 1];
            }

            // Cost: smoothness (finite differences) + obstacle
            double smooth = 0.0;
            for (int k = 1; k < N-1; ++k) {
                double ax = qPert[k-1][0] - 2.0*qPert[k][0] + qPert[k+1][0];
                double ay = qPert[k-1][1] - 2.0*qPert[k][1] + qPert[k+1][1];
                smooth += 0.5 * (ax*ax + ay*ay);
            }
            double obs = 0.0;
            for (int k = 0; k < N; ++k) {
                obs += obstacleCost(qPert[k]);
            }
            costs[m] = smooth + lambda * obs;
        }

        // Soft-min weights
        double cmin = Double.POSITIVE_INFINITY;
        for (double c : costs) {
            if (c < cmin) cmin = c;
        }
        double[] w = new double[numRollouts];
        double wsum = 0.0;
        for (int m = 0; m < numRollouts; ++m) {
            w[m] = Math.exp(-(costs[m] - cmin) / eta);
            wsum += w[m];
        }
        for (int m = 0; m < numRollouts; ++m) {
            w[m] /= wsum;
        }

        // Update
        double[] delta = new double[N * dim];
        for (int m = 0; m < numRollouts; ++m) {
            for (int i = 0; i < N * dim; ++i) {
                delta[i] += w[m] * eps[m][i];
            }
        }

        double[][] qNew = new double[N][dim];
        for (int k = 0; k < N; ++k) {
            qNew[k][0] = q[k][0] + delta[2*k];
            qNew[k][1] = q[k][1] + delta[2*k + 1];
        }
        return qNew;
    }
}
      
