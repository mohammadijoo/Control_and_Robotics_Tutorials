public interface ConfigSpace<C> {
    double distance(C a, C b);
    C interpolate(C a, C b, double alpha); // alpha in [0, 1]
    boolean collisionFree(C a, C b);
    C sampleUniform();
    C sampleInformed(double cBest, C start, C goal);
}

public class RRTStarNode<C> {
    public C x;
    public RRTStarNode<C> parent;
    public double g;
    public RRTStarNode(C x, RRTStarNode<C> parent, double g) {
        this.x = x;
        this.parent = parent;
        this.g = g;
    }
}

public class InformedRRTStar<C> {
    private final ConfigSpace<C> space;
    private final C start, goal;
    private final double step;
    private final double goalThreshold;
    private final java.util.List<RRTStarNode<C>> tree;
    private double cBest;

    public InformedRRTStar(ConfigSpace<C> space, C start, C goal,
                           double step, double goalThreshold) {
        this.space = space;
        this.start = start;
        this.goal = goal;
        this.step = step;
        this.goalThreshold = goalThreshold;
        this.tree = new java.util.ArrayList<>();
        this.tree.add(new RRTStarNode<>(start, null, 0.0));
        this.cBest = Double.POSITIVE_INFINITY;
    }

    private RRTStarNode<C> nearest(C x) {
        RRTStarNode<C> best = null;
        double bestDist = Double.POSITIVE_INFINITY;
        for (RRTStarNode<C> n : tree) {
            double d = space.distance(n.x, x);
            if (d < bestDist) {
                bestDist = d;
                best = n;
            }
        }
        return best;
    }

    public RRTStarNode<C> plan(int maxIterations, double radius) {
        RRTStarNode<C> bestGoal = null;
        for (int k = 0; k < maxIterations; ++k) {
            C xRand = space.sampleInformed(cBest, start, goal);
            RRTStarNode<C> xNear = nearest(xRand);

            double d = space.distance(xNear.x, xRand);
            double alpha = Math.min(1.0, step / Math.max(d, 1e-9));
            C xNew = space.interpolate(xNear.x, xRand, alpha);

            if (!space.collisionFree(xNear.x, xNew)) continue;

            // Compute cost-to-come candidate
            double gNew = xNear.g + space.distance(xNear.x, xNew);

            // Choose parent and rewire neighbors inside radius
            java.util.List<RRTStarNode<C>> neighbors = new java.util.ArrayList<>();
            for (RRTStarNode<C> n : tree) {
                if (space.distance(n.x, xNew) <= radius)
                    neighbors.add(n);
            }
            RRTStarNode<C> xParent = xNear;
            double gMin = gNew;
            for (RRTStarNode<C> n : neighbors) {
                double gCand = n.g + space.distance(n.x, xNew);
                if (gCand < gMin && space.collisionFree(n.x, xNew)) {
                    gMin = gCand;
                    xParent = n;
                }
            }
            RRTStarNode<C> xNode = new RRTStarNode<>(xNew, xParent, gMin);
            tree.add(xNode);

            // Rewire
            for (RRTStarNode<C> n : neighbors) {
                double gCand = gMin + space.distance(xNew, n.x);
                if (gCand + 1e-9 < n.g && space.collisionFree(xNew, n.x)) {
                    n.parent = xNode;
                    n.g = gCand;
                }
            }

            // Check for goal
            if (space.distance(xNew, goal) <= goalThreshold
                    && space.collisionFree(xNew, goal)) {
                double gGoal = gMin + space.distance(xNew, goal);
                if (gGoal < cBest) {
                    cBest = gGoal;
                    bestGoal = new RRTStarNode<>(goal, xNode, gGoal);
                    tree.add(bestGoal);
                }
            }
        }
        return bestGoal;
    }
}
      
