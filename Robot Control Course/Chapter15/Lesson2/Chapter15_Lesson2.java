
public class ResidualModel {
    private final int outDim;
    private final int inDim;
    private final double[][] W;
    private final double eta;

    public ResidualModel(int outDim, int inDim, double eta) {
        this.outDim = outDim;
        this.inDim = inDim;
        this.eta = eta;
        this.W = new double[outDim][inDim];
    }

    public double[] predict(double[] phi) {
        double[] y = new double[outDim];
        for (int i = 0; i < outDim; ++i) {
            double s = 0.0;
            for (int j = 0; j < inDim; ++j) {
                s += W[i][j] * phi[j];
            }
            y[i] = s;
        }
        return y;
    }

    public void update(double[] phi, double[] target) {
        double[] err = predict(phi);
        for (int i = 0; i < outDim; ++i) {
            err[i] -= target[i];
        }
        for (int i = 0; i < outDim; ++i) {
            for (int j = 0; j < inDim; ++j) {
                W[i][j] -= eta * err[i] * phi[j];
            }
        }
    }
}
