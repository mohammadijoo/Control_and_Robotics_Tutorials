public interface TaskPolicy {
    // state: e.g., joint positions and velocities
    // context: task embedding (goal pose, ID, etc.)
    double[] act(double[] state, double[] context);
}

public class LinearGeneralistPolicy implements TaskPolicy {
    private final int stateDim;
    private final int contextDim;
    private final int actionDim;
    private final double[][] W;
    private final double[] b;

    public LinearGeneralistPolicy(int stateDim, int contextDim, int actionDim) {
        this.stateDim = stateDim;
        this.contextDim = contextDim;
        this.actionDim = actionDim;
        this.W = new double[actionDim][stateDim + contextDim];
        this.b = new double[actionDim];
        initRandom();
    }

    private void initRandom() {
        java.util.Random rng = new java.util.Random(0);
        for (int i = 0; i < actionDim; ++i) {
            for (int j = 0; j < stateDim + contextDim; ++j) {
                W[i][j] = 0.1 * rng.nextGaussian();
            }
            b[i] = 0.0;
        }
    }

    @Override
    public double[] act(double[] state, double[] context) {
        double[] z = new double[stateDim + contextDim];
        System.arraycopy(state, 0, z, 0, stateDim);
        System.arraycopy(context, 0, z, stateDim, contextDim);

        double[] action = new double[actionDim];
        for (int i = 0; i < actionDim; ++i) {
            double s = b[i];
            for (int j = 0; j < z.length; ++j) {
                s += W[i][j] * z[j];
            }
            action[i] = s;
        }
        return action;
    }
}
      
