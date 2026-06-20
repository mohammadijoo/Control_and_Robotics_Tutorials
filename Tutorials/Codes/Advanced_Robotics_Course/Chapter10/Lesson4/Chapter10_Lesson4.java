public class LogisticAffordance {
    private final double[] theta;
    private final double lr;
    private final double reg;

    public LogisticAffordance(int d, double lr, double reg) {
        this.theta = new double[d];
        this.lr = lr;
        this.reg = reg;
    }

    private double sigmoid(double z) {
        return 1.0 / (1.0 + Math.exp(-z));
    }

    public double predictProba(double[] phi) {
        double z = 0.0;
        for (int i = 0; i != phi.length; ++i) {
            z += theta[i] * phi[i];
        }
        return sigmoid(z);
    }

    public void trainBatch(double[][] Phi, int[] y) {
        int N = Phi.length;
        int d = theta.length;
        double[] grad = new double[d];

        for (int i = 0; i != N; ++i) {
            double p = predictProba(Phi[i]);
            double diff = y[i] - p;
            for (int j = 0; j != d; ++j) {
                grad[j] += -diff * Phi[i][j];
            }
        }

        for (int j = 0; j != d; ++j) {
            grad[j] /= (double) N;
            grad[j] += reg * theta[j];
            theta[j] -= lr * grad[j];
        }
    }

    public double[] getTheta() {
        return theta;
    }
}
      
