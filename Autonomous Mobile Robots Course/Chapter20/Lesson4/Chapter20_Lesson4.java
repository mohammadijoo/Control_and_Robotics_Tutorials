// Chapter20_Lesson4.java
// Navigation Stack Deployment (Capstone AMR)
// Java educational deployment manager and monitoring logic.
import java.util.*;

public class Chapter20_Lesson4 {

    enum NodeState { UNCONFIGURED, INACTIVE, ACTIVE, ERROR }

    static class ManagedNode {
        String name;
        double startupMs;
        NodeState state = NodeState.UNCONFIGURED;

        ManagedNode(String name, double startupMs) {
            this.name = name;
            this.startupMs = startupMs;
        }

        boolean configure() {
            state = NodeState.INACTIVE;
            return true;
        }

        boolean activate() {
            if (state != NodeState.INACTIVE) {
                state = NodeState.ERROR;
                return false;
            }
            state = NodeState.ACTIVE;
            return true;
        }

        void deactivate() {
            if (state == NodeState.ACTIVE) state = NodeState.INACTIVE;
        }

        void reset() { state = NodeState.UNCONFIGURED; }
    }

    static class TimingBudget {
        double periodMs;
        Map<String, Double> wcetMs = new LinkedHashMap<>();

        TimingBudget(double periodMs) { this.periodMs = periodMs; }

        double utilization() {
            double u = 0.0;
            for (double c : wcetMs.values()) u += c / periodMs;
            return u;
        }

        boolean isSchedulable() { return utilization() <= 1.0; }
    }

    static class DeploymentSupervisor {
        List<ManagedNode> nodes;
        TimingBudget timing;
        double covTraceLimit = 0.20 * 0.20;
        int plannerMissCount = 0;
        int maxConsecutiveMisses = 3;
        double plannerDeadlineMs = 80.0;

        DeploymentSupervisor(List<ManagedNode> nodes, TimingBudget timing) {
            this.nodes = nodes;
            this.timing = timing;
        }

        boolean startup() {
            if (!timing.isSchedulable()) {
                System.out.println("Timing budget invalid");
                return false;
            }
            for (ManagedNode n : nodes) if (!n.configure()) return false;
            for (ManagedNode n : nodes) if (!n.activate()) return false;
            System.out.println("Startup complete");
            return true;
        }

        boolean checkLocalizationHealth(double covXX, double covYY) {
            double traceXY = covXX + covYY;
            boolean ok = traceXY <= covTraceLimit;
            System.out.printf(Locale.US, "Localization trace=%.4f => %s%n", traceXY, ok ? "OK" : "RECOVERY");
            return ok;
        }

        boolean monitorPlannerDeadline(double runtimeMs) {
            if (runtimeMs > plannerDeadlineMs) plannerMissCount++;
            else plannerMissCount = 0;

            if (plannerMissCount >= maxConsecutiveMisses) {
                System.out.println("Recovery trigger: clear costmap and stop");
                plannerMissCount = 0;
                return false;
            }
            return true;
        }

        void shutdown() {
            ListIterator<ManagedNode> it = nodes.listIterator(nodes.size());
            while (it.hasPrevious()) {
                ManagedNode n = it.previous();
                n.deactivate();
                n.reset();
            }
            System.out.println("Shutdown complete");
        }
    }

    static int inflatedCost(double d, double rIns, double rInf, int cLethal, int cIns, double kappa) {
        if (d <= rIns) return cLethal;
        if (d >= rInf) return 0;
        double v = cIns * Math.exp(-kappa * (d - rIns));
        v = Math.max(1.0, Math.min(cIns, v));
        return (int) Math.round(v);
    }

    static double scoreTrajectory(
        double distPath, double distGoal, int obsCost, double v, double omega, double vRef,
        double wPath, double wGoal, double wObs, double wVel, double wSpin
    ) {
        double obsTerm = ((double) obsCost) / 254.0;
        return wPath * distPath + wGoal * distGoal + wObs * obsTerm + wVel * Math.abs(v - vRef) + wSpin * Math.abs(omega);
    }

    public static void main(String[] args) {
        TimingBudget budget = new TimingBudget(100.0);
        budget.wcetMs.put("sensor_preprocess", 8.0);
        budget.wcetMs.put("localization", 14.0);
        budget.wcetMs.put("costmap_update", 18.0);
        budget.wcetMs.put("global_planner", 12.0);
        budget.wcetMs.put("local_planner", 22.0);
        budget.wcetMs.put("controller", 5.0);
        budget.wcetMs.put("bt_tick", 3.0);

        System.out.printf(Locale.US, "Utilization = %.3f, schedulable=%s%n", budget.utilization(), budget.isSchedulable());

        List<ManagedNode> nodes = Arrays.asList(
            new ManagedNode("map_server", 20),
            new ManagedNode("localization", 25),
            new ManagedNode("global_planner", 15),
            new ManagedNode("local_planner", 12),
            new ManagedNode("controller_server", 10),
            new ManagedNode("bt_navigator", 18)
        );

        DeploymentSupervisor sup = new DeploymentSupervisor(nodes, budget);
        if (!sup.startup()) return;

        double[] dSamples = {0.18, 0.25, 0.35, 0.50, 0.80};
        for (double d : dSamples) {
            System.out.println("inflation cost(" + d + ") = " + inflatedCost(d, 0.22, 0.70, 254, 220, 8.0));
        }

        double[][] candidates = {
            {0.05, 1.20, 140, 0.40, 0.10},
            {0.18, 0.95,  80, 0.55, 0.35},
            {0.10, 1.00, 220, 0.50, 0.05}
        };
        int bestIdx = -1;
        double bestScore = Double.POSITIVE_INFINITY;
        for (int i = 0; i < candidates.length; i++) {
            double s = scoreTrajectory(
                candidates[i][0], candidates[i][1], (int) candidates[i][2], candidates[i][3], candidates[i][4], 0.50,
                1.4, 1.0, 2.6, 0.4, 0.2
            );
            System.out.printf(Locale.US, "Candidate %d score = %.4f%n", i, s);
            if (s < bestScore) { bestScore = s; bestIdx = i; }
        }
        System.out.println("Best candidate = " + bestIdx);

        double[][] covs = {
            {0.006, 0.007}, {0.007, 0.006}, {0.008, 0.005},
            {0.010, 0.009}, {0.012, 0.011}, {0.035, 0.034},
            {0.009, 0.007}, {0.008, 0.008}, {0.007, 0.006}
        };
        double[] runtimes = {60, 72, 91, 88, 95, 65, 77, 83, 90};

        for (int k = 0; k < runtimes.length; k++) {
            if (!sup.checkLocalizationHealth(covs[k][0], covs[k][1])) {
                System.out.println("Action: reduce speed and relocalize");
            }
            sup.monitorPlannerDeadline(runtimes[k]);
        }

        sup.shutdown();
    }
}
