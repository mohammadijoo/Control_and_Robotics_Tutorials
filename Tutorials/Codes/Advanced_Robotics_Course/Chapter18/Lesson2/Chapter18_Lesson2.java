import java.util.*;
import java.util.stream.Collectors;

class Box2D {
    public double xmin, ymin, xmax, ymax;
    public Box2D(double xmin, double ymin, double xmax, double ymax) {
        this.xmin = xmin; this.ymin = ymin;
        this.xmax = xmax; this.ymax = ymax;
    }
}

class PlanningTask2D {
    public double[] start;
    public double[] goal;
    public List<Box2D> obstacles;
    public String id;
    public int stratumId;

    public PlanningTask2D(double[] start, double[] goal, List<Box2D> obstacles,
                          String id, int stratumId) {
        this.start = start;
        this.goal = goal;
        this.obstacles = obstacles;
        this.id = id;
        this.stratumId = stratumId;
    }
}

interface Planner {
    // Returns true if a collision-free solution is found within the time limit.
    boolean solve(PlanningTask2D task, double timeLimitSeconds);
}

class BenchmarkResult {
    public String plannerName;
    public String taskId;
    public boolean success;
    public double solveTime;

    public BenchmarkResult(String plannerName, String taskId,
                           boolean success, double solveTime) {
        this.plannerName = plannerName;
        this.taskId = taskId;
        this.success = success;
        this.solveTime = solveTime;
    }
}

class BenchmarkHarness {
    private List<PlanningTask2D> tasks;
    private List<Planner> planners;
    private Map<Planner,String> plannerNames;

    public BenchmarkHarness(List<PlanningTask2D> tasks) {
        this.tasks = tasks;
        this.planners = new ArrayList<>();
        this.plannerNames = new HashMap<>();
    }

    public void addPlanner(Planner planner, String name) {
        planners.add(planner);
        plannerNames.put(planner, name);
    }

    public List<BenchmarkResult> run(double timeLimitSeconds, int numRepeats) {
        List<BenchmarkResult> results = new ArrayList<>();
        for (Planner planner : planners) {
            for (PlanningTask2D task : tasks) {
                for (int r = 0; r < numRepeats; ++r) {
                    long t0 = System.nanoTime();
                    boolean success = planner.solve(task, timeLimitSeconds);
                    long t1 = System.nanoTime();
                    double elapsed = (t1 - t0) * 1e-9;
                    results.add(new BenchmarkResult(
                        plannerNames.get(planner),
                        task.id,
                        success,
                        elapsed
                    ));
                }
            }
        }
        return results;
    }

    public Map<String, Double> computeSuccessRates(List<BenchmarkResult> results) {
        Map<String, List<BenchmarkResult>> byPlanner =
            results.stream().collect(Collectors.groupingBy(r -> r.plannerName));
        Map<String, Double> success = new HashMap<>();
        for (String p : byPlanner.keySet()) {
            List<BenchmarkResult> rs = byPlanner.get(p);
            long numSuccess = rs.stream().filter(r -> r.success).count();
            success.put(p, numSuccess / (double) rs.size());
        }
        return success;
    }
}
      
