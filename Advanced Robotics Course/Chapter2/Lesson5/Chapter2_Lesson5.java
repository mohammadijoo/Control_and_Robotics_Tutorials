import java.util.*;
import java.util.stream.Collectors;

public class ManipAStar {

    public static final int DOF = 7;

    static class JointLimit {
        double qMin, qMax, delta;
        JointLimit(double qMin, double qMax, double delta) {
            this.qMin = qMin; this.qMax = qMax; this.delta = delta;
        }
    }

    static JointLimit[] LIMITS = new JointLimit[] {
        new JointLimit(-2.9, 2.9, 0.1),
        new JointLimit(-2.0, 2.0, 0.1),
        new JointLimit(-2.0, 2.0, 0.1),
        new JointLimit(-2.9, 2.9, 0.1),
        new JointLimit(-2.0, 2.0, 0.1),
        new JointLimit(-2.0, 2.0, 0.1),
        new JointLimit(-3.1, 3.1, 0.1)
    };

    static class Node {
        int[] k;
        Node(int[] k) {
            this.k = k;
        }
        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Node)) return false;
            return Arrays.equals(k, ((Node)o).k);
        }
        @Override
        public int hashCode() {
            return Arrays.hashCode(k);
        }
    }

    static Node discretize(double[] q) {
        int[] k = new int[DOF];
        for (int i = 0; i < DOF; ++i) {
            JointLimit lim = LIMITS[i];
            int idx = (int)Math.round((q[i] - lim.qMin) / lim.delta);
            int maxK = (int)Math.round((lim.qMax - lim.qMin) / lim.delta);
            if (idx < 0) idx = 0;
            if (idx > maxK) idx = maxK;
            k[i] = idx;
        }
        return new Node(k);
    }

    static double[] undisc(Node n) {
        double[] q = new double[DOF];
        for (int i = 0; i < DOF; ++i) {
            JointLimit lim = LIMITS[i];
            q[i] = lim.qMin + n.k[i] * lim.delta;
        }
        return q;
    }

    static boolean isCollisionFree(double[] q) {
        // TODO: integrate with a collision checker (e.g., via rosjava and MoveIt).
        return true;
    }

    static List<Node> neighbors(Node n) {
        List<Node> nbrs = new ArrayList<>();
        for (int i = 0; i < DOF; ++i) {
            for (int step : new int[]{-1, 1}) {
                int[] kk = Arrays.copyOf(n.k, DOF);
                kk[i] += step;
                JointLimit lim = LIMITS[i];
                int maxK = (int)Math.round((lim.qMax - lim.qMin) / lim.delta);
                if (kk[i] < 0 || kk[i] > maxK) continue;
                double[] q = undisc(new Node(kk));
                if (isCollisionFree(q)) {
                    nbrs.add(new Node(kk));
                }
            }
        }
        return nbrs;
    }

    static double edgeCost(Node a, Node b) {
        double[] qa = undisc(a);
        double[] qb = undisc(b);
        double s = 0.0;
        for (int i = 0; i < DOF; ++i) {
            s += Math.abs(qb[i] - qa[i]);
        }
        return s;
    }

    static double h(Node n, Node goal) {
        double[] q = undisc(n);
        double[] qg = undisc(goal);
        double s = 0.0;
        for (int i = 0; i < DOF; ++i) {
            double d = q[i] - qg[i];
            s += d * d;
        }
        return Math.sqrt(s);
    }

    public static List<double[]> astar(double[] qStart, double[] qGoal) {
        Node start = discretize(qStart);
        Node goal = discretize(qGoal);

        PriorityQueue<Node> open = new PriorityQueue<>(
            Comparator.comparingDouble(n -> gScore.getOrDefault(n, Double.POSITIVE_INFINITY)
                    + h(n, goal))
        );
        open.add(start);

        gScore = new HashMap<>();
        parent = new HashMap<>();
        gScore.put(start, 0.0);

        while (!open.isEmpty()) {
            Node current = open.poll();
            if (current.equals(goal)) {
                List<Node> pathNodes = new ArrayList<>();
                pathNodes.add(current);
                while (parent.containsKey(current)) {
                    current = parent.get(current);
                    pathNodes.add(current);
                }
                Collections.reverse(pathNodes);
                return pathNodes.stream()
                        .map(ManipAStar::undisc)
                        .collect(Collectors.toList());
            }

            for (Node nb : neighbors(current)) {
                double tentativeG = gScore.get(current) + edgeCost(current, nb);
                double oldG = gScore.getOrDefault(nb, Double.POSITIVE_INFINITY);
                if (tentativeG < oldG) {
                    gScore.put(nb, tentativeG);
                    parent.put(nb, current);
                    if (!open.contains(nb)) {
                        open.add(nb);
                    }
                }
            }
        }
        throw new RuntimeException("No path found");
    }

    private static Map<Node, Double> gScore;
    private static Map<Node, Node> parent;

    public static void main(String[] args) {
        double[] qStart = {0.0, -1.0, 0.5, 0.0, 0.0, 0.5, 0.0};
        double[] qGoal  = {1.0,  0.5, 0.0, 0.5, 0.0, 1.0, 0.0};
        List<double[]> path = astar(qStart, qGoal);
        System.out.println("Path length: " + path.size());
    }
}
      
