public class LinearActor {
    private final int obsDim;
    private final int actDim;
    private final double[][] W;  // actDim x obsDim
    private final double[] b;
    private double logStd = Math.log(0.1);

    public LinearActor(int obsDim, int actDim) {
        this.obsDim = obsDim;
        this.actDim = actDim;
        this.W = new double[actDim][obsDim];
        this.b = new double[actDim];
    }

    public double[] mean(double[] s) {
        double[] mu = new double[actDim];
        for (int i = 0; i < actDim; i++) {
            double v = b[i];
            for (int j = 0; j < obsDim; j++) {
                v += W[i][j] * s[j];
            }
            mu[i] = v;
        }
        return mu;
    }

    public double[] sample(double[] s, java.util.Random rng,
                           double[] outLogp) {
        double[] mu = mean(s);
        double std = Math.exp(logStd);
        double[] a = new double[actDim];
        double logp = 0.0;
        for (int i = 0; i < actDim; i++) {
            double eps = rng.nextGaussian() * std;
            a[i] = mu[i] + eps;
            double diff = a[i] - mu[i];
            logp += -0.5 * Math.log(2.0 * Math.PI * std * std)
                    - 0.5 * diff * diff / (std * std);
        }
        outLogp[0] = logp;
        return a;
    }

    public void update(double[] s, double[] a, double advantage,
                       double alpha) {
        double std = Math.exp(logStd);
        double invVar = 1.0 / (std * std);
        double[] mu = mean(s);
        for (int i = 0; i < actDim; i++) {
            double diff = (a[i] - mu[i]) * invVar;
            for (int j = 0; j < obsDim; j++) {
                W[i][j] += alpha * advantage * diff * s[j];
            }
            b[i] += alpha * advantage * diff;
        }
    }
}
      
