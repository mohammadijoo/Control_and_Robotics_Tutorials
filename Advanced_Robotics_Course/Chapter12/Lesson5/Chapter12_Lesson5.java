public interface SafeEnvironment {
    double[] reset();
    StepResult step(double[] action);
    int getObservationDim();
    int getActionDim();
    // StepResult contains obs, reward, done, constraintCost
}

public class SafeRLAgent {
    private int obsDim;
    private int actDim;
    private double lambdaC = 0.0;
    private double lambdaLr = 1e-2;
    private double costLimit = 1.0;

    // Parameters of policy and value functions would live here
    // (e.g., INDArray weights if using ND4J).

    public SafeRLAgent(int obsDim, int actDim) {
        this.obsDim = obsDim;
        this.actDim = actDim;
    }

    public double[] act(double[] obs) {
        // Sample from Gaussian policy (stochastic exploration).
        double[] mu = policyMean(obs);
        double[] std = policyStd(obs);
        double[] action = new double[actDim];
        for (int i = 0; i < actDim; i++) {
            action[i] = mu[i] + std[i] * randomNormal();
        }
        return action;
    }

    public void updateOnTrajectory(List<double[]> obsTraj,
                                   List<double[]> actTraj,
                                   List<Double> rewTraj,
                                   List<Double> costTraj,
                                   double gamma)
    {
        int T = rewTraj.size();
        double[] G_r = new double[T];
        double[] G_c = new double[T];
        double gR = 0.0, gC = 0.0;
        for (int t = T - 1; t >= 0; t--) {
            gR = rewTraj.get(t) + gamma * gR;
            gC = costTraj.get(t) + gamma * gC;
            G_r[t] = gR;
            G_c[t] = gC;
        }

        // Compute gradients and update policy parameters here.
        // For brevity we omit ND4J tensor operations.

        double avgCost = 0.0;
        for (double c : costTraj) {
            avgCost += c;
        }
        avgCost /= (double) T;

        // Dual update
        lambdaC = Math.max(0.0, lambdaC + lambdaLr * (avgCost - costLimit));
    }

    private double[] policyMean(double[] obs) {
        // Placeholder linear policy
        double[] mu = new double[actDim];
        for (int i = 0; i < actDim; i++) {
            mu[i] = 0.0;
        }
        return mu;
    }

    private double[] policyStd(double[] obs) {
        double[] std = new double[actDim];
        for (int i = 0; i < actDim; i++) {
            std[i] = 0.2; // fixed std
        }
        return std;
    }

    private double randomNormal() {
        return new java.util.Random().nextGaussian();
    }
}
      
