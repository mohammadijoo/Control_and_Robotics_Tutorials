public interface Simulator {
    SimState reset(long noiseSeed);
    SimState step(double[] u);
    int getStateDim();
}

public class SimState {
    public double[] state;
    public double[] sensor;
    public double[] goal;
    public boolean done;
    public boolean collision;
    public boolean taskSuccess;
    public double cost;
}

public interface Perception {
    void reset();
    double[] update(double[] sensor);
}

public interface Planner {
    double[] plan(double[] belief, double[] goal);
}

public interface Controller {
    void reset();
    double[] computeControl(double[] xRef, double[] belief);
}

public class Metrics {
    public double J;
    public boolean success;
    public boolean violation;
}

public class Evaluation {

    public static Metrics runEpisode(
            Simulator sim,
            Perception perc,
            Planner planner,
            Controller ctrl,
            long noiseSeed,
            int horizon) {

        SimState state = sim.reset(noiseSeed);
        perc.reset();
        ctrl.reset();

        double J = 0.0;
        boolean violated = false;
        boolean success = false;

        for (int t = 0; t < horizon; t++) {
            double[] belief = perc.update(state.sensor);
            double[] xRef = planner.plan(belief, state.goal);
            double[] u = ctrl.computeControl(xRef, belief);
            state = sim.step(u);

            J += state.cost;
            if (state.collision) {
                violated = true;
            }
            if (state.done) {
                success = !violated && state.taskSuccess;
                break;
            }
        }

        Metrics m = new Metrics();
        m.J = J;
        m.success = success;
        m.violation = violated;
        return m;
    }

    public static double[] evaluateStack(
            Simulator sim,
            Perception perc,
            Planner planner,
            Controller ctrl,
            int N,
            int horizon) {

        java.util.Random rng = new java.util.Random(0L);

        double[] Js = new double[N];
        double[] succ = new double[N];
        double[] viol = new double[N];

        for (int i = 0; i < N; i++) {
            long seed = rng.nextLong();
            Metrics m = runEpisode(sim, perc, planner, ctrl, seed, horizon);
            Js[i] = m.J;
            succ[i] = m.success ? 1.0 : 0.0;
            viol[i] = m.violation ? 1.0 : 0.0;
        }

        double[] res = new double[6];
        res[0] = mean(Js);
        res[1] = stdOverSqrtN(Js);
        res[2] = mean(succ);
        res[3] = stdOverSqrtN(succ);
        res[4] = mean(viol);
        res[5] = stdOverSqrtN(viol);
        return res;
    }

    private static double mean(double[] xs) {
        double s = 0.0;
        for (double x : xs) s += x;
        return s / xs.length;
    }

    private static double stdOverSqrtN(double[] xs) {
        double m = mean(xs);
        double v = 0.0;
        for (double x : xs) {
            double d = x - m;
            v += d * d;
        }
        v /= (xs.length - 1);
        return Math.sqrt(v / xs.length);
    }
}
      
