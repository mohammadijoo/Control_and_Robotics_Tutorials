public interface Planner {
    Plan computePlan(State x, Goal g);
}

public class RRTStarPlanner implements Planner {
    @Override
    public Plan computePlan(State x, Goal g) {
        // RRT* planning logic
        return new Plan();
    }
}

public class TrajOptPlanner implements Planner {
    @Override
    public Plan computePlan(State x, Goal g) {
        // Trajectory optimization logic
        return new Plan();
    }
}

public class AutonomyStack {
    private final Planner planner;
    private final Controller controller;
    private final Estimator estimator;

    public AutonomyStack(Planner planner,
                         Controller controller,
                         Estimator estimator) {
        this.planner = planner;
        this.controller = controller;
        this.estimator = estimator;
    }

    public ControlOutput step(SensorMsg msg, Goal goal) {
        State xhat = estimator.update(msg);
        Plan plan = planner.computePlan(xhat, goal);
        ControlOutput u = controller.compute(xhat, plan);
        return u;
    }
}
      
