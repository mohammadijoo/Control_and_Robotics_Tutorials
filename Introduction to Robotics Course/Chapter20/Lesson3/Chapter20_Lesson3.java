import java.util.Arrays;

public class MilestoneCondition {

    // C is m x n, x is length n, d is length m
    private final double[][] C;
    private final double[] d;

    public MilestoneCondition(double[][] C, double[] d) {
        this.C = C;
        this.d = d;
    }

    public boolean isReached(double[] x) {
        for (int i = 0; i != C.length; ++i) {
            double lhs = 0.0;
            for (int j = 0; j != x.length; ++j) {
                lhs += C[i][j] * x[j];
            }
            if (lhs < d[i]) {
                return false;
            }
        }
        return true;
    }

    public static void main(String[] args) {
        // Example: two subsystems, milestone when both >= 0.8
        double[][] C = {
          {1.0, 0.0}, 
          {0.0, 1.0}
        };
        double[] d = {0.8, 0.8};
        MilestoneCondition M = new MilestoneCondition(C, d);

        double[] x1 = {0.6, 0.9};
        double[] x2 = {0.85, 0.9};

        System.out.println("Reached at x1? " + M.isReached(x1));
        System.out.println("Reached at x2? " + M.isReached(x2));
    }
}
      
