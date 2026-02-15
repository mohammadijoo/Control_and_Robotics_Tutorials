public class TOPP1D {

    public static class Result {
        public double[] s;
        public double[] sdot;
        public double[] t;
    }

    public static Result topp1D(double sF, int N, double vMax, double aMax) {
        if (N <= 0) {
            throw new IllegalArgumentException("N must be positive");
        }

        Result res = new Result();
        res.s = new double[N + 1];
        res.sdot = new double[N + 1];
        res.t = new double[N + 1];

        double ds = sF / (double) N;
        for (int k = 0; k <= N; ++k) {
            res.s[k] = ds * k;
            res.sdot[k] = 0.0;
            res.t[k] = 0.0;
        }

        double[] sdotMax = new double[N + 1];
        for (int k = 0; k <= N; ++k) {
            sdotMax[k] = vMax;
        }

        // Forward pass
        for (int k = 0; k < N; ++k) {
            double sdotSq = res.sdot[k] * res.sdot[k] + 2.0 * aMax * ds;
            double sdotCand = Math.sqrt(Math.max(0.0, sdotSq));
            res.sdot[k + 1] = Math.min(sdotCand, sdotMax[k + 1]);
        }

        // Backward pass
        for (int k = N - 1; k >= 0; --k) {
            double sdotSq = res.sdot[k + 1] * res.sdot[k + 1] + 2.0 * aMax * ds;
            double sdotCand = Math.sqrt(Math.max(0.0, sdotSq));
            double min = Math.min(res.sdot[k], sdotCand);
            res.sdot[k] = Math.min(min, sdotMax[k]);
        }

        // Time stamps
        for (int k = 0; k < N; ++k) {
            double denom = res.sdot[k] + res.sdot[k + 1];
            if (denom <= 1e-9) {
                throw new RuntimeException("Infeasible profile: zero velocity segment");
            }
            res.t[k + 1] = res.t[k] + 2.0 * ds / denom;
        }

        return res;
    }

    // Example usage:
    // public static void main(String[] args) {
    //     Result res = topp1D(1.0, 100, 1.0, 2.0);
    //     System.out.println("Final time T = " + res.t[res.t.length - 1]);
    // }
}
      
