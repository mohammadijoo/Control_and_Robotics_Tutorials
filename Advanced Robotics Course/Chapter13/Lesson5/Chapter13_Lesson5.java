import java.util.Random;
import java.util.ArrayList;
import java.util.List;

public class TransferEvaluator {

    public static class Params {
        public double massScale;
        public double frictionScale;
        public double latency;
    }

    public static class EpisodeStats {
        public double ret;
        public boolean success;
        public EpisodeStats(double r, boolean s) {
            this.ret = r;
            this.success = s;
        }
    }

    public static class RandomizationConfig {
        private double mMin, mMax, fMin, fMax, lMin, lMax;
        private Random rng;

        public RandomizationConfig(double mMin, double mMax,
                                   double fMin, double fMax,
                                   double lMin, double lMax,
                                   long seed) {
            this.mMin = mMin;
            this.mMax = mMax;
            this.fMin = fMin;
            this.fMax = fMax;
            this.lMin = lMin;
            this.lMax = lMax;
            this.rng = new Random(seed);
        }

        public Params sample() {
            Params p = new Params();
            p.massScale = mMin + rng.nextDouble() * (mMax - mMin);
            p.frictionScale = fMin + rng.nextDouble() * (fMax - fMin);
            p.latency = lMin + rng.nextDouble() * (lMax - lMin);
            return p;
        }
    }

    public static EpisodeStats runEpisodeSim(RandomizedEnv env,
                                             Policy policy,
                                             Params params,
                                             int maxSteps) {
        env.reset(params);
        double totalReward = 0.0;
        boolean success = false;

        for (int t = 0; t < maxSteps; ++t) {
            double[] obs = env.observe();
            double[] action = policy.act(obs);
            StepResult step = env.step(action);
            totalReward += step.getReward();
            if (step.getInfo().isSuccess()) {
                success = true;
            }
            if (step.isDone()) {
                break;
            }
        }
        return new EpisodeStats(totalReward, success);
    }

    public static EpisodeStats runEpisodeReal(Policy policy,
                                              Params params,
                                              int maxSteps) {
        // Wrap hardware control; safety constraints must be handled externally.
        return RealRobot.runEpisode(policy, params, maxSteps);
    }

    public static void main(String[] args) {
        RandomizedEnv env = new BulletRandomizedEnv();
        Policy policy = PolicyLoader.load("policy.bin");

        RandomizationConfig evalCfg = new RandomizationConfig(
                0.9, 1.1,
                0.8, 1.2,
                0.0, 0.03,
                42L
        );

        final int N = 40;
        final int maxSteps = 200;

        List<Double> simReturns = new ArrayList<>();
        List<Double> realReturns = new ArrayList<>();
        List<Integer> simSuccess = new ArrayList<>();
        List<Integer> realSuccess = new ArrayList<>();

        for (int i = 0; i < N; ++i) {
            Params p = evalCfg.sample();
            EpisodeStats esSim = runEpisodeSim(env, policy, p, maxSteps);
            EpisodeStats esReal = runEpisodeReal(policy, p, maxSteps);

            simReturns.add(esSim.ret);
            realReturns.add(esReal.ret);
            simSuccess.add(esSim.success ? 1 : 0);
            realSuccess.add(esReal.success ? 1 : 0);
        }

        double JSim = mean(simReturns);
        double JReal = mean(realReturns);
        double pSim = meanInt(simSuccess);
        double pReal = meanInt(realSuccess);

        System.out.println("Expected return (sim, real): " + JSim + ", " + JReal);
        System.out.println("Success prob. (sim, real): " + pSim + ", " + pReal);
        System.out.println("Transfer gap (return): " + (JReal - JSim));
        System.out.println("Transfer gap (success): " + (pReal - pSim));
    }

    private static double mean(List<Double> v) {
        double s = 0.0;
        for (double x : v) s += x;
        return s / v.size();
    }

    private static double meanInt(List<Integer> v) {
        double s = 0.0;
        for (int x : v) s += x;
        return s / v.size();
    }
}
      
