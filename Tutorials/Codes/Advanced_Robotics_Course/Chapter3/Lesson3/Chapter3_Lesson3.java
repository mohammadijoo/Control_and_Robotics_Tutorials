import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class Config2D {
    public double x, y;
    public Config2D(double x, double y) { this.x = x; this.y = y; }
    public Config2D add(Config2D other) { return new Config2D(x + other.x, y + other.y); }
    public Config2D sub(Config2D other) { return new Config2D(x - other.x, y - other.y); }
    public Config2D mul(double s) { return new Config2D(s * x, s * y); }
    public double norm() { return Math.sqrt(x * x + y * y); }
}

class Node {
    public Config2D q;
    public int parent; // -1 for root
    public Node(Config2D q, int parent) { this.q = q; this.parent = parent; }
}

interface CollisionChecker {
    boolean collisionFree(Config2D q1, Config2D q2);
}

class Tree {
    public List<Node> nodes = new ArrayList<>();

    public int addNode(Config2D q, int parent) {
        nodes.add(new Node(q, parent));
        return nodes.size() - 1;
    }

    public int nearest(Config2D q) {
        double best = Double.POSITIVE_INFINITY;
        int bestIdx = -1;
        for (int i = 0; i < nodes.size(); ++i) {
            Config2D qi = nodes.get(i).q;
            double dx = qi.x - q.x;
            double dy = qi.y - q.y;
            double d = Math.sqrt(dx * dx + dy * dy);
            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }
}

public class RRT2D {
    private Random rng = new Random();
    private double xmin, xmax, ymin, ymax;

    public RRT2D(double xmin, double xmax, double ymin, double ymax) {
        this.xmin = xmin; this.xmax = xmax;
        this.ymin = ymin; this.ymax = ymax;
    }

    private Config2D sample() {
        double x = xmin + rng.nextDouble() * (xmax - xmin);
        double y = ymin + rng.nextDouble() * (ymax - ymin);
        return new Config2D(x, y);
    }

    private Config2D steer(Config2D qNear, Config2D qRand, double eta) {
        Config2D d = qRand.sub(qNear);
        double dist = d.norm();
        if (dist <= eta) return qRand;
        return qNear.add(d.mul(eta / dist));
    }

    public List<Config2D> plan(Config2D qStart,
                                 Config2D qGoal,
                                 CollisionChecker checker,
                                 double eta,
                                 double goalRadius,
                                 int maxIter) {
        Tree tree = new Tree();
        tree.addNode(qStart, -1);

        for (int k = 0; k < maxIter; ++k) {
            Config2D qRand = (rng.nextDouble() < 0.1) ? qGoal : sample();
            int idxNear = tree.nearest(qRand);
            Config2D qNear = tree.nodes.get(idxNear).q;
            Config2D qNew = steer(qNear, qRand, eta);
            if (checker.collisionFree(qNear, qNew)) {
                int idxNew = tree.addNode(qNew, idxNear);
                double dx = qNew.x - qGoal.x;
                double dy = qNew.y - qGoal.y;
                if (Math.sqrt(dx * dx + dy * dy) <= goalRadius) {
                    // reconstruct
                    ArrayList<Config2D> path = new ArrayList<>();
                    int cur = idxNew;
                    while (cur != -1) {
                        path.add(0, tree.nodes.get(cur).q);
                        cur = tree.nodes.get(cur).parent;
                    }
                    return path;
                }
            }
        }
        return null; // failure
    }
}
      
