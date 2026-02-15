import java.util.Random;

public class PlanarArmEnvDR {
    private final Random rng;
    private final double[] massNominal = {1.0, 1.0};
    private final double massRelRange = 0.3;
    private final double frictionNominal = 0.5;
    private final double frictionRelRange = 0.5;

    private double[] linkMasses = new double[2];
    private double contactFriction;

    public PlanarArmEnvDR(long seed) {
        this.rng = new Random(seed);
    }

    private double uniform(double low, double high) {
        return low + (high - low) * rng.nextDouble();
    }

    private void sampleParameters() {
        for (int i = 0; i < 2; ++i) {
            double low = (1.0 - massRelRange) * massNominal[i];
            double high = (1.0 + massRelRange) * massNominal[i];
            linkMasses[i] = uniform(low, high);
        }
        double lowf = (1.0 - frictionRelRange) * frictionNominal;
        double highf = (1.0 + frictionRelRange) * frictionNominal;
        contactFriction = uniform(lowf, highf);
    }

    public double[] reset() {
        sampleParameters();
        // Here one would call engine APIs, e.g.:
        // engine.setLinkMass(0, linkMasses[0]);
        // engine.setLinkMass(1, linkMasses[1]);
        // engine.setContactFriction(contactFriction);

        // Return initial state (here, all zeros)
        return new double[]{0.0, 0.0, 0.0, 0.0};
    }

    public StepResult step(double[] action) {
        // Call simulator integration with current randomized params
        // ...
        double[] nextState = new double[]{0.0, 0.0, 0.0, 0.0};
        double reward = 0.0;
        boolean done = false;
        return new StepResult(nextState, reward, done);
    }

    public static class StepResult {
        public final double[] nextState;
        public final double reward;
        public final boolean done;

        public StepResult(double[] nextState, double reward, boolean done) {
            this.nextState = nextState;
            this.reward = reward;
            this.done = done;
        }
    }
}
      
