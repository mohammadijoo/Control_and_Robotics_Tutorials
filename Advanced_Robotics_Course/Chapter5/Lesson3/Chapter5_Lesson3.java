import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class State {
    double x, y, vx, vy;
    State(double x, double y, double vx, double vy) {
        this.x = x; this.y = y; this.vx = vx; this.vy = vy;
    }
}

class Node {
    State state;
    Node parent;
    double cost;
    Node(State s, Node p, double c) {
        this.state = s; this.parent = p; this.cost = c;
    }
}

public class KinodynamicRRTStar {
    private final List<Node> tree = new ArrayList<>();
    private final Random rand = new Random(1234);

    double distance(State a, State b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dvx = a.vx - b.vx;
        double dvy = a.vy - b.vy;
        double wPos = 1.0, wVel = 0.1;
        return Math.sqrt(wPos * (dx * dx + dy * dy)
                       + wVel * (dvx * dvx + dvy * dvy));
    }

    State propagate(State s, double ux, double uy, double dt, int steps) {
        double x = s.x, y = s.y, vx = s.vx, vy = s.vy;
        for (int k = 0; k < steps; ++k) {
            x  += dt * vx;
            y  += dt * vy;
            vx += dt * ux;
            vy += dt * uy;
        }
        return new State(x, y, vx, vy);
    }

    public void run(State init, int maxIter) {
        tree.add(new Node(init, null, 0.0));

        double xmin = 0.0, xmax = 10.0;
        double ymin = 0.0, ymax = 10.0;
        double vmax = 2.0;
        double[][] uSet = {
            {-1.0, 0.0}, {1.0, 0.0},
            {0.0, -1.0}, {0.0, 1.0}, {0.0, 0.0}
        };
        double dt = 0.1;
        int steps = 5;
        int dim = 4;
        double gamma = 30.0;

        for (int n = 1; n <= maxIter; ++n) {
            State xRand = new State(
                xmin + (xmax - xmin) * rand.nextDouble(),
                ymin + (ymax - ymin) * rand.nextDouble(),
                -vmax + 2.0 * vmax * rand.nextDouble(),
                -vmax + 2.0 * vmax * rand.nextDouble()
            );

            // nearest
            Node nearest = tree.get(0);
            double best = Double.POSITIVE_INFINITY;
            for (Node node : tree) {
                double d = distance(node.state, xRand);
                if (d < best) {
                    best = d;
                    nearest = node;
                }
            }

            State xNew = null;
            double bestEdgeCost = Double.POSITIVE_INFINITY;
            for (double[] u : uSet) {
                State cand = propagate(nearest.state, u[0], u[1], dt, steps);
                // bounds check (collision omitted)
                if (cand.x < xmin || cand.x > xmax ||
                    cand.y < ymin || cand.y > ymax) {
                    continue;
                }
                double edgeCost = dt * steps;
                if (edgeCost < bestEdgeCost) {
                    bestEdgeCost = edgeCost;
                    xNew = cand;
                }
            }
            if (xNew == null) continue;

            // radius (not used here for simplicity)
            double rn = Math.pow(
                gamma * Math.log((double)n) / (double)n,
                1.0 / (double)dim
            );

            Node newNode = new Node(xNew, nearest,
                                    nearest.cost + bestEdgeCost);
            tree.add(newNode);
            // neighbor-based rewiring omitted
        }
    }

    public static void main(String[] args) {
        KinodynamicRRTStar planner = new KinodynamicRRTStar();
        planner.run(new State(1.0, 1.0, 0.0, 0.0), 500);
        System.out.println("Tree size: " + planner.tree.size());
    }
}
      
