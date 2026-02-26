public class BeliefState {
    private final int S;
    private double[] belief;

    // T[a][s][sNext]
    private final double[][][] T;
    // Z[a][sNext][o]
    private final double[][][] Z;

    public BeliefState(double[] initialBelief,
                       double[][][] T,
                       double[][][] Z) {
        this.S = initialBelief.length;
        this.belief = initialBelief.clone();
        this.T = T;
        this.Z = Z;
    }

    public double[] getBelief() {
        return belief.clone();
    }

    public void update(int action, int observation) {
        double[] bp = new double[S];
        for (int s = 0; s < S; ++s) {
            for (int sp = 0; sp < S; ++sp) {
                bp[sp] += T[action][s][sp] * belief[s];
            }
        }
        double[] bnew = new double[S];
        double norm = 0.0;
        for (int sp = 0; sp < S; ++sp) {
            bnew[sp] = Z[action][sp][observation] * bp[sp];
            norm += bnew[sp];
        }
        if (norm < 1e-12) {
            for (int sp = 0; sp < S; ++sp) {
                bnew[sp] = 1.0 / S;
            }
        } else {
            for (int sp = 0; sp < S; ++sp) {
                bnew[sp] /= norm;
            }
        }
        belief = bnew;
    }

    // Example policy: QMDP-like argmax_a sum_s b(s) Q[s][a]
    public int selectAction(double[][] Q) {
        int A = Q[0].length;
        double bestVal = Double.NEGATIVE_INFINITY;
        int bestA = 0;
        for (int a = 0; a < A; ++a) {
            double qa = 0.0;
            for (int s = 0; s < S; ++s) {
                qa += belief[s] * Q[s][a];
            }
            if (qa > bestVal) {
                bestVal = qa;
                bestA = a;
            }
        }
        return bestA;
    }
}
      
