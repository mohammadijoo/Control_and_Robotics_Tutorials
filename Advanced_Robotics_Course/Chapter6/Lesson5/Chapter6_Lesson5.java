public class RiskSensitiveMDP {
    private int nStates;
    private int nActions;
    private double gamma;
    private double lambdaRisk;
    // P[s][a][s']
    private double[][][] P;
    private double[][] cost;

    public RiskSensitiveMDP(int nStates, int nActions, double gamma, double lambdaRisk) {
        this.nStates = nStates;
        this.nActions = nActions;
        this.gamma = gamma;
        this.lambdaRisk = lambdaRisk;
        this.P = new double[nStates][nActions][nStates];
        this.cost = new double[nStates][nActions];
    }

    public void setTransition(int s, int a, int sp, double prob) {
        P[s][a][sp] = prob;
    }

    public void setCost(int s, int a, double c) {
        cost[s][a] = c;
    }

    public void valueIteration(double[] V, int[] policy, int maxIter, double tol) {
        double[] Vnew = new double[nStates];
        for (int s = 0; s < nStates; ++s) V[s] = 0.0;

        for (int it = 0; it < maxIter; ++it) {
            double diff = 0.0;
            for (int s = 0; s < nStates; ++s) {
                double best = Double.POSITIVE_INFINITY;
                for (int a = 0; a < nActions; ++a) {
                    double expTerm = 0.0;
                    for (int sp = 0; sp < nStates; ++sp) {
                        double p = P[s][a][sp];
                        if (p > 0.0) {
                            expTerm += p * Math.exp(
                                lambdaRisk * (cost[s][a] + gamma * V[sp])
                            );
                        }
                    }
                    double q = (1.0 / lambdaRisk) * Math.log(expTerm);
                    if (q < best) best = q;
                }
                Vnew[s] = best;
                diff = Math.max(diff, Math.abs(Vnew[s] - V[s]));
            }
            System.arraycopy(Vnew, 0, V, 0, nStates);
            if (diff < tol) break;
        }

        for (int s = 0; s < nStates; ++s) {
            double best = Double.POSITIVE_INFINITY;
            int bestA = 0;
            for (int a = 0; a < nActions; ++a) {
                double expTerm = 0.0;
                for (int sp = 0; sp < nStates; ++sp) {
                    double p = P[s][a][sp];
                    if (p > 0.0) {
                        expTerm += p * Math.exp(
                            lambdaRisk * (cost[s][a] + gamma * V[sp])
                        );
                    }
                }
                double q = (1.0 / lambdaRisk) * Math.log(expTerm);
                if (q < best) { best = q; bestA = a; }
            }
            policy[s] = bestA;
        }
    }
}
      
