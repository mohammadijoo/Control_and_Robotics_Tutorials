public interface TrajectoryRequirement {
    boolean check(double[] t, double[] y);
}

public class ReachGoalWithinTime implements TrajectoryRequirement {
    private final double yGoal;
    private final double epsilon;
    private final double tMax;

    public ReachGoalWithinTime(double yGoal, double epsilon, double tMax) {
        this.yGoal = yGoal;
        this.epsilon = epsilon;
        this.tMax = tMax;
    }

    @Override
    public boolean check(double[] t, double[] y) {
        for (int k = 0; k < t.length; ++k) {
            if (t[k] > tMax) {
                break;
            }
            double e = Math.abs(y[k] - yGoal);
            if (e <= epsilon) {
                return true; // requirement satisfied
            }
        }
        return false;
    }
}

// Example usage in a simulation context
public class RequirementDemo {
    public static void main(String[] args) {
        int N = 1000;
        double[] t = new double[N];
        double[] y = new double[N];
        double T = 2.0;

        // Simple ramp trajectory y(t) = t, sampled uniformly
        for (int k = 0; k < N; ++k) {
            t[k] = T * k / (N - 1);
            y[k] = t[k];
        }

        TrajectoryRequirement req =
            new ReachGoalWithinTime(1.0, 0.01, 1.5);

        boolean ok = req.check(t, y);
        System.out.println("Reachability requirement: " + (ok ? "OK" : "VIOLATED"));
    }
}
      
