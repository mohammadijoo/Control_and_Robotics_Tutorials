public class OneDPOMDP {
    // belief[0] = P(state 0), belief[1] = P(state 1)
    private double[] belief = new double[]{0.5, 0.5};

    // Transition matrices T[action][s][s_prime]
    private final double[][][] T = new double[][][]{
        {   // "stay"
            {0.9, 0.0},
            {0.1, 1.0}
        },
        {   // "step"
            {0.1, 0.0},
            {0.9, 1.0}
        }
    };

    // Observation model Z[o_index][s_prime]
    private final double[][] Z = new double[][]{
        {0.2, 0.9},  // "door"
        {0.8, 0.1}   // "no-door"
    };

    private int actionIndex(String a) {
        if (a.equals("stay")) return 0;
        if (a.equals("step")) return 1;
        throw new IllegalArgumentException("Unknown action");
    }

    private int obsIndex(String o) {
        if (o.equals("door")) return 0;
        if (o.equals("no-door")) return 1;
        throw new IllegalArgumentException("Unknown observation");
    }

    private double[] predict(double[] b, int aIdx) {
        double[] out = new double[]{0.0, 0.0};
        for (int s = 0; s < 2; ++s) {
            for (int sp = 0; sp < 2; ++sp) {
                out[sp] += T[aIdx][s][sp] * b[s];
            }
        }
        return out;
    }

    private void normalize(double[] v) {
        double sum = v[0] + v[1];
        if (sum == 0.0) {
            v[0] = 0.5;
            v[1] = 0.5;
        } else {
            v[0] /= sum;
            v[1] /= sum;
        }
    }

    public void applyActionAndObservation(String a, String o) {
        int aIdx = actionIndex(a);
        int oIdx = obsIndex(o);
        double[] bPred = predict(belief, aIdx);
        double[] bPost = new double[2];
        bPost[0] = Z[oIdx][0] * bPred[0];
        bPost[1] = Z[oIdx][1] * bPred[1];
        normalize(bPost);
        belief = bPost;
    }

    public double[] getBelief() {
        return belief;
    }

    public static void main(String[] args) {
        OneDPOMDP pomdp = new OneDPOMDP();
        pomdp.applyActionAndObservation("step", "door");
        double[] b = pomdp.getBelief();
        System.out.println("Belief: " + b[0] + ", " + b[1]);
    }
}
      
