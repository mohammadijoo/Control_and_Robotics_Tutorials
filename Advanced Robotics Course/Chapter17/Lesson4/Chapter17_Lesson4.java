public class LinearDeformablePolicy {
    private final int obsDim;
    private final int actDim;
    private final double[][] W; // [actDim][obsDim]
    private final double lr;

    public LinearDeformablePolicy(int obsDim, int actDim, double lr) {
        this.obsDim = obsDim;
        this.actDim = actDim;
        this.lr = lr;
        this.W = new double[actDim][obsDim];
        // initialize small random weights
        java.util.Random rnd = new java.util.Random(0);
        for (int i = 0; i < actDim; ++i) {
            for (int j = 0; j < obsDim; ++j) {
                W[i][j] = 0.01 * rnd.nextGaussian();
            }
        }
    }

    // Forward pass: u = W * obs
    public double[] act(double[] obs) {
        double[] u = new double[actDim];
        for (int i = 0; i < actDim; ++i) {
            double sum = 0.0;
            for (int j = 0; j < obsDim; ++j) {
                sum += W[i][j] * obs[j];
            }
            u[i] = sum;
        }
        return u;
    }

    // Single gradient descent step on squared loss
    public void trainStep(double[][] obsBatch, double[][] uStarBatch) {
        int batchSize = obsBatch.length;
        // accumulate gradients dL/dW
        double[][] gradW = new double[actDim][obsDim];

        for (int k = 0; k < batchSize; ++k) {
            double[] obs = obsBatch[k];
            double[] uStar = uStarBatch[k];
            double[] uPred = act(obs);

            for (int i = 0; i < actDim; ++i) {
                double error = uPred[i] - uStar[i];
                for (int j = 0; j < obsDim; ++j) {
                    gradW[i][j] += error * obs[j];
                }
            }
        }

        // update W
        double scale = lr / batchSize;
        for (int i = 0; i < actDim; ++i) {
            for (int j = 0; j < obsDim; ++j) {
                W[i][j] -= scale * gradW[i][j];
            }
        }
    }
}
      
