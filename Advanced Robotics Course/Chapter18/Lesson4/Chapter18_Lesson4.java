import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;
import java.util.Random;

public class ReproducibleEvaluator {

    private final RobotEnvironment env;
    private final Random rng;
    private final int numEpisodes;
    private final int maxSteps;

    public ReproducibleEvaluator(RobotEnvironment env,
                                 long globalSeed,
                                 int numEpisodes,
                                 int maxSteps) {
        this.env = env;
        this.rng = new Random(globalSeed);
        this.numEpisodes = numEpisodes;
        this.maxSteps = maxSteps;
    }

    public void run(String logPath) throws IOException {
        try (FileWriter writer = new FileWriter(logPath)) {
            writer.write("episode,totalCost,success\n");
            for (int episode = 0; episode < numEpisodes; episode++) {
                EpisodeResult res = runEpisode(episode);
                writer.write(episode + "," + res.totalCost + "," + res.success + "\n");
            }
        }
    }

    private EpisodeResult runEpisode(int episodeIndex) {
        env.reset(rng, episodeIndex); // deterministic given rng state and index
        double totalCost = 0.0;
        boolean success = false;

        for (int t = 0; t < maxSteps; t++) {
            double[] obs = env.getObservation();
            double[] action = scriptedPolicy(obs);
            // Inject small, reproducible perturbation
            for (int i = 0; i < action.length; i++) {
                action[i] += 0.01 * rng.nextGaussian();
            }
            StepResult step = env.step(action);
            totalCost += step.cost;
            if (step.success) {
                success = true;
            }
            if (step.terminated) {
                break;
            }
        }
        return new EpisodeResult(totalCost, success);
    }

    private double[] scriptedPolicy(double[] obs) {
        // Deterministic policy; fill with your control logic
        double[] action = new double[env.getActionDim()];
        for (int i = 0; i < action.length; i++) {
            action[i] = 0.0;
        }
        return action;
    }

    private static class EpisodeResult {
        final double totalCost;
        final boolean success;
        EpisodeResult(double totalCost, boolean success) {
            this.totalCost = totalCost;
            this.success = success;
        }
    }

    public static void main(String[] args) throws IOException {
        // RobotEnvironment would be implemented using ROSJava, WPILib, or a custom simulator.
        RobotEnvironment env = new MyRobotEnvironment();
        ReproducibleEvaluator evaluator =
                new ReproducibleEvaluator(env, 1234L, 50, 200);
        evaluator.run("java_repro_results.csv");
    }
}
      
