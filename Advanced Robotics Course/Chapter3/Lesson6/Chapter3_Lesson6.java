import java.util.*;
import java.util.stream.Collectors;

class Config6D {
    double[] q = new double[6];

    Config6D(double[] q) {
        if (q.length != 6) throw new IllegalArgumentException("q must be length 6");
        System.arraycopy(q, 0, this.q, 0, 6);
    }
}

class Edge {
    int u, v;
    double w;
    Edge(int u, int v, double w) {
        this.u = u;
        this.v = v;
        this.w = w;
    }
}

interface CollisionChecker {
    boolean inCollision(double[] q);
    boolean edgeCollisionFree(double[] q1, double[] q2);
}

public class PRM6D {
    private final double[] qMin;
    private final double[] qMax;
    private final CollisionChecker checker;
    private final Random rng = new Random();

    public PRM6D(double[] qMin, double[] qMax, CollisionChecker checker) {
        if (qMin.length != 6 || qMax.length != 6)
            throw new IllegalArgumentException("Joint limits must be length 6");
        this.qMin = qMin.clone();
        this.qMax = qMax.clone();
        this.checker = checker;
    }

    private double[] sampleConfig() {
        double[] q = new double[6];
        for (int i = 0; i < 6; ++i) {
            q[i] = qMin[i] + (qMax[i] - qMin[i]) * rng.nextDouble();
        }
        return q;
    }

    private double distance(double[] q1, double[] q2) {
        double s = 0.0;
        for (int i = 0; i < 6; ++i) {
            double d = q1[i] - q2[i];
            s += d * d;
        }
        return Math.sqrt(s);
    }

    public List<double[]> plan(double[] qStart, double[] qGoal,
                               int nSamples, int kNeighbors) {
        List<Config6D> nodes = new ArrayList<>();
        // Add PRM samples
        for (int i = 0; i < nSamples; ++i) {
            double[] q = sampleConfig();
            if (!checker.inCollision(q)) {
                nodes.add(new Config6D(q));
            }
        }
        // Add start and goal
        int startIdx = nodes.size();
        nodes.add(new Config6D(qStart.clone()));
        int goalIdx = nodes.size();
        nodes.add(new Config6D(qGoal.clone()));

        int n = nodes.size();
        List<List<Edge>> adj = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) adj.add(new ArrayList<>());

        // Connect PRM nodes
        for (int i = 0; i < n - 2; ++i) {
            List<int[]> neigh = new ArrayList<>();
            for (int j = 0; j < n - 2; ++j) {
                if (i == j) continue;
                double d = distance(nodes.get(i).q, nodes.get(j).q);
                neigh.add(new int[]{j, Double.valueOf(d).hashCode()});
            }
            // sort by distance
            neigh.sort(Comparator.comparingInt(a -> a[1]));
            int count = 0;
            for (int[] pair : neigh) {
                if (count >= kNeighbors) break;
                int j = pair[0];
                if (checker.edgeCollisionFree(nodes.get(i).q, nodes.get(j).q)) {
                    double w = distance(nodes.get(i).q, nodes.get(j).q);
                    adj.get(i).add(new Edge(i, j, w));
                    adj.get(j).add(new Edge(j, i, w));
                    count++;
                }
            }
        }

        // Connect start and goal to nearest neighbors
        connectSpecial(nodes, adj, startIdx, kNeighbors);
        connectSpecial(nodes, adj, goalIdx, kNeighbors);

        // Dijkstra shortest path
        double[] dist = new double[n];
        int[] parent = new int[n];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        dist[startIdx] = 0.0;

        PriorityQueue<int[]> pq =
            new PriorityQueue<>(Comparator.comparingDouble(a -> dist[a[0]]));
        pq.add(new int[]{startIdx});

        while (!pq.isEmpty()) {
            int u = pq.poll()[0];
            if (u == goalIdx) break;
            for (Edge e : adj.get(u)) {
                int v = e.v;
                double nd = dist[u] + e.w;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    parent[v] = u;
                    pq.add(new int[]{v});
                }
            }
        }

        if (!Double.isFinite(dist[goalIdx])) {
            return null;
        }

        List<double[]> path = new ArrayList<>();
        int idx = goalIdx;
        while (idx != -1) {
            path.add(nodes.get(idx).q);
            idx = parent[idx];
        }
        Collections.reverse(path);
        return path;
    }

    private void connectSpecial(List<Config6D> nodes,
                                List<List<Edge>> adj,
                                int idxSpecial, int kNeighbors) {
        int n = nodes.size();
        List<double[]> candidates = new ArrayList<>();
        for (int i = 0; i < n - 2; ++i) {
            if (i == idxSpecial) continue;
            candidates.add(new double[]{i, distance(nodes.get(idxSpecial).q, nodes.get(i).q)});
        }
        candidates.sort(Comparator.comparingDouble(a -> a[1]));
        int count = 0;
        for (double[] pair : candidates) {
            if (count >= kNeighbors) break;
            int j = (int) pair[0];
            if (checker.edgeCollisionFree(nodes.get(idxSpecial).q, nodes.get(j).q)) {
                double w = pair[1];
                adj.get(idxSpecial).add(new Edge(idxSpecial, j, w));
                adj.get(j).add(new Edge(j, idxSpecial, w));
                count++;
            }
        }
    }
}
      
