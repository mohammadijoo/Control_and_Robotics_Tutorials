public interface TaskEnvironment {
    Observation reset();
    StepResult step(Action action);
}

public class StepResult {
    public final Observation observation;
    public final double reward;
    public final boolean done;
    public final boolean isSuccess;

    public StepResult(Observation obs, double reward, boolean done, boolean isSuccess) {
        this.observation = obs;
        this.reward = reward;
        this.done = done;
        this.isSuccess = isSuccess;
    }
}

public interface Policy {
    Action act(Observation observation);
}

public class EvalStats {
    public double successRate;
    public double avgCost;
}

public class Evaluator {
    public static EvalStats evaluate(Policy policy,
                                     java.util.List<TaskEnvironment> tasks,
                                     int episodesPerTask,
                                     int maxHorizon) {
        int totalEpisodes = 0;
        int totalSuccess = 0;
        double totalCost = 0.0;

        for (TaskEnvironment task : tasks) {
            for (int i = 0; i < episodesPerTask; i++) {
                Observation obs = task.reset();
                boolean done = false;
                int step = 0;
                double episodeCost = 0.0;

                while (!done && step < maxHorizon) {
                    Action action = policy.act(obs);
                    StepResult result = task.step(action);
                    obs = result.observation;
                    done = result.done;
                    episodeCost += -result.reward;
                    step++;
                    if (done && result.isSuccess) {
                        totalSuccess++;
                    }
                }
                totalCost += episodeCost;
                totalEpisodes++;
            }
        }
        EvalStats stats = new EvalStats();
        stats.successRate = (double) totalSuccess / Math.max(totalEpisodes, 1);
        stats.avgCost = totalCost / Math.max(totalEpisodes, 1);
        return stats;
    }
}
      
