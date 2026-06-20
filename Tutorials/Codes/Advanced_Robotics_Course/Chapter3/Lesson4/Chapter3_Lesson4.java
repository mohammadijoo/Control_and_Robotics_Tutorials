import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class Node {
    double x, y;
    int parent;
    double cost;
    Node(double x, double y, int parent, double cost) {
        this.x = x;
        this.y = y;
        this.parent = parent;
        this.cost = cost;
    }
}

public class RRTStar2D {
    private double xMin, xMax, yMin, yMax;
    private double stepSize;
    private double goalRadius;
    private int maxIter;
    private double[][] obstacles; // each row: [xmin, ymin, xmax, ymax]
    private List<Node> nodes;
    private double[] goal;
    private Random rng;

    public RRTStar2D(double xMin, double xMax,
                     double yMin, double yMax,
                     double[] start, double[] goal,
                     double stepSize,
                     double goalRadius,
                     int maxIter) {
        this.xMin = xMin; this.xMax = xMax;
        this.yMin = yMin; this.yMax = yMax;
        this.stepSize = stepSize;
        this.goalRadius = goalRadius;
        this.maxIter = maxIter;
        this.nodes = new ArrayList<>();
        this.nodes.add(new Node(start[0], start[1], -1, 0.0));
        this.goal = goal.clone();
        this.rng = new Random(0);
        this.obstacles = new double[0][4];
    }

    public void setObstacles(double[][] obs) {
        this.obstacles = obs;
    }

    private double[] sampleFree() {
        double x = xMin + rng.nextDouble() * (xMax - xMin);
        double y = yMin + rng.nextDouble() * (yMax - yMin);
        return new double[]{x, y};
    }

    private double dist(double[] p, double[] q) {
        double dx = p[0] - q[0];
        double dy = p[1] - q[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    private int nearest(double[] xRand) {
        double best = Double.POSITIVE_INFINITY;
        int idx = 0;
        for (int i = 0; i < nodes.size(); ++i) {
            Node n = nodes.get(i);
            double d = dist(new double[]{n.x, n.y}, xRand);
            if (d < best) {
                best = d;
                idx = i;
            }
        }
        return idx;
    }

    private double[] steer(double[] xNearest, double[] xRand) {
        double dx = xRand[0] - xNearest[0];
        double dy = xRand[1] - xNearest[1];
        double norm = Math.sqrt(dx * dx + dy * dy);
        if (norm <= stepSize) {
            return xRand.clone();
        }
        double scale = stepSize / norm;
        return new double[]{
            xNearest[0] + scale * dx,
            xNearest[1] + scale * dy
        };
    }

    private boolean inObstacle(double[] p) {
        double px = p[0], py = p[1];
        for (double[] obs : obstacles) {
            if (px >= obs[0] && px <= obs[2] &&
                py >= obs[1] && py <= obs[3]) {
                return true;
            }
        }
        return false;
    }

    private boolean lineCollisionFree(double[] p, double[] q, int steps) {
        for (int i = 0; i <= steps; ++i) {
            double alpha = (double) i / (double) steps;
            double x = (1.0 - alpha) * p[0] + alpha * q[0];
            double y = (1.0 - alpha) * p[1] + alpha * q[1];
            if (inObstacle(new double[]{x, y})) {
                return false;
            }
        }
        return true;
    }

    private List<Integer> near(double[] xNew, double gammaRrt) {
        int n = nodes.size();
        List<Integer> idxs = new ArrayList<>();
        if (n == 1) {
            idxs.add(0);
            return idxs;
        }
        int d = 2;
        double rN = gammaRrt * Math.pow(Math.log(n) / (double) n, 1.0 / d);
        rN = Math.min(rN, stepSize * 10.0);
        for (int i = 0; i < n; ++i) {
            Node node = nodes.get(i);
            double dxy = dist(new double[]{node.x, node.y}, xNew);
            if (dxy <= rN) {
                idxs.add(i);
            }
        }
        return idxs;
    }

    public List<double[]> plan() {
        Integer goalIdx = null;
        for (int k = 0; k < maxIter; ++k) {
            double[] xRand = sampleFree();
            int idxNearest = nearest(xRand);
            Node xNear = nodes.get(idxNearest);
            double[] xNearArr = new double[]{xNear.x, xNear.y};
            double[] xNew = steer(xNearArr, xRand);
            if (inObstacle(xNew)) continue;
            if (!lineCollisionFree(xNearArr, xNew, 10)) continue;

            List<Integer> neighbors = near(xNew, 1.0);
            int bestParent = idxNearest;
            double bestCost = xNear.cost + dist(xNearArr, xNew);
            for (int i : neighbors) {
                Node ni = nodes.get(i);
                double[] xi = new double[]{ni.x, ni.y};
                if (!lineCollisionFree(xi, xNew, 10)) continue;
                double cand = ni.cost + dist(xi, xNew);
                if (cand < bestCost) {
                    bestCost = cand;
                    bestParent = i;
                }
            }
            Node newNode = new Node(xNew[0], xNew[1], bestParent, bestCost);
            nodes.add(newNode);
            int newIdx = nodes.size() - 1;

            // rewire
            for (int i : neighbors) {
                if (i == bestParent) continue;
                Node ni = nodes.get(i);
                double[] xi = new double[]{ni.x, ni.y};
                if (!lineCollisionFree(xNew, xi, 10)) continue;
                double cand = bestCost + dist(xNew, xi);
                if (cand < ni.cost) {
                    ni.parent = newIdx;
                    ni.cost = cand;
                }
            }

            // goal check
            if (dist(xNew, goal) <= goalRadius) {
                if (goalIdx == null ||
                    bestCost + dist(xNew, goal)
                        < nodes.get(goalIdx).cost) {
                    goalIdx = newIdx;
                }
            }
        }
        return extractPath(goalIdx);
    }

    private List<double[]> extractPath(Integer goalIdx) {
        List<double[]> path = new ArrayList<>();
        if (goalIdx == null) return path;
        path.add(goal.clone());
        int idx = goalIdx;
        while (idx != -1) {
            Node n = nodes.get(idx);
            path.add(0, new double[]{n.x, n.y});
            idx = n.parent;
        }
        return path;
    }
}
      
