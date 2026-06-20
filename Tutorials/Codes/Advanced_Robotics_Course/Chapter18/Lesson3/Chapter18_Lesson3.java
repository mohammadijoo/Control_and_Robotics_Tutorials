import java.util.Random;

public class StressTest {

    private static final Random rng = new Random(42L);

    // Sample stress scenarios (simplified distributions).
    static double[] sampleScenario() {
        double friction = Math.max(0.0, Math.min(1.0, rng.nextGaussian() * 0.2 + 0.3));
        double mass = Math.exp(rng.nextGaussian() * 0.25);  // log-normal-ish
        double latency = -0.08 * Math.log(1.0 - rng.nextDouble());
        return new double[]{friction, mass, latency};
    }

    static class TrialResult {
        boolean failed;
        String label;
        TrialResult(boolean f, String l) { failed = f; label = l; }
    }

    static TrialResult simulateTrial(double friction, double mass, double latency) {
        boolean stoppingViolation = (friction * mass) < 0.6;
        boolean latencyViolation = latency > 0.15;
        boolean timeoutViolation = (mass > 2.5) && (latency > 0.10);

        if (stoppingViolation && latencyViolation) {
            return new TrialResult(true, "control+environment:multi-factor");
        }
        if (stoppingViolation) {
            return new TrialResult(true, "control:insufficient_braking");
        }
        if (latencyViolation) {
            return new TrialResult(true, "control:latency_instability");
        }
        if (timeoutViolation) {
            return new TrialResult(true, "performance:timeout");
        }
        return new TrialResult(false, "success");
    }

    public static void main(String[] args) {
        int N = 5000;
        double delta = 0.05;
        int failures = 0;

        for (int i = 0; i < N; ++i) {
            double[] s = sampleScenario();
            TrialResult tr = simulateTrial(s[0], s[1], s[2]);
            if (tr.failed) {
                failures++;
                // In practice, increment a Map<String,Integer> of taxonomy counts.
            }
        }

        double pHat = (double) failures / N;
        double eps = Math.sqrt(Math.log(2.0 / delta) / (2.0 * N));
        double ciLow = Math.max(0.0, pHat - eps);
        double ciHigh = Math.min(1.0, pHat + eps);

        System.out.println("N = " + N);
        System.out.println("Estimated failure probability: " + pHat);
        System.out.println("Hoeffding 95% CI: [" + ciLow + ", " + ciHigh + "]");
    }
}
      
