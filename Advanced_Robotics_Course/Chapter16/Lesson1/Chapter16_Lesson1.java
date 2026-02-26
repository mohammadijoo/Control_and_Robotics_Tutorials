import java.util.*;

public class CorridorSafetyChecker {
    static class StateStep {
        final double x;
        final int step;

        StateStep(double x, int step) {
            this.x = x;
            this.step = step;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof StateStep)) return false;
            StateStep other = (StateStep) o;
            return Math.round(100 * x) == Math.round(100 * other.x)
                   && step == other.step;
        }

        @Override
        public int hashCode() {
            long xi = Math.round(100 * x);
            return Objects.hash(xi, step);
        }
    }

    public static void main(String[] args) {
        double T = 0.1;
        double L = 1.0;
        double u_max = 0.5;
        int N_horizon = 10;
        double dx = 0.1;

        double[] controls = new double[]{-u_max, 0.0, u_max};
        double[] initialStates = new double[]{0.0};

        Queue<StateStep> queue = new ArrayDeque<>();
        Set<StateStep> visited = new HashSet<>();

        for (double x0 : initialStates) {
            StateStep ss = new StateStep(x0, 0);
            queue.add(ss);
            visited.add(ss);
        }

        boolean safe = true;

        while (!queue.isEmpty() && safe) {
            StateStep node = queue.poll();
            double x = node.x;
            int step = node.step;

            if (step == N_horizon) continue;

            for (double u : controls) {
                double xNext = x + T * u;
                if (xNext < -L || xNext > L) {
                    System.out.println("Unsafe successor from x=" + x
                                       + " with u=" + u);
                    safe = false;
                    break;
                }
                double xDisc = dx * Math.round(xNext / dx);
                StateStep nextNode = new StateStep(xDisc, step + 1);
                if (!visited.contains(nextNode)) {
                    visited.add(nextNode);
                    queue.add(nextNode);
                }
            }
        }

        System.out.println("Abstraction declared SAFE up to horizon "
                           + N_horizon + ": " + safe);
    }
}
      
