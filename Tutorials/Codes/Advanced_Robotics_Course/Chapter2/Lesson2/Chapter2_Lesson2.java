import java.util.*;

class Edge {
    public final int to;
    public final double cost;
    public Edge(int to, double cost) {
        this.to = to;
        this.cost = cost;
    }
}

public class AStar {
    public static class NodeRecord implements Comparable<NodeRecord> {
        public final int v;
        public final double f;
        public final double g;
        public NodeRecord(int v, double f, double g) {
            this.v = v; this.f = f; this.g = g;
        }
        @Override
        public int compareTo(NodeRecord other) {
            return Double.compare(this.f, other.f);
        }
    }

    public static class Result {
        public final double cost;
        public final List<Integer> path;
        public Result(double cost, List<Integer> path) {
            this.cost = cost; this.path = path;
        }
    }

    public static Result astar(
        List<List<Edge>> graph,
        int start,
        int goal,
        java.util.function.DoubleUnaryOperator heuristic)
    {
        int n = graph.size();
        double[] g = new double[n];
        int[] parent = new int[n];
        boolean[] closed = new boolean[n];
        Arrays.fill(g, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);
        PriorityQueue<NodeRecord> open = new PriorityQueue<>();

        g[start] = 0.0;
        open.add(new NodeRecord(start, heuristic.applyAsDouble(start), 0.0));

        while (!open.isEmpty()) {
            NodeRecord rec = open.poll();
            int v = rec.v;
            if (closed[v]) continue;
            if (v == goal) {
                LinkedList<Integer> path = new LinkedList<>();
                for (int u = goal; u != -1; u = parent[u]) {
                    path.addFirst(u);
                }
                return new Result(rec.g, path);
            }
            closed[v] = true;
            for (Edge e : graph.get(v)) {
                if (e.cost < 0.0)
                    throw new IllegalArgumentException("Negative edge cost");
                double gTent = rec.g + e.cost;
                if (gTent < g[e.to]) {
                    g[e.to] = gTent;
                    parent[e.to] = v;
                    double f = gTent + heuristic.applyAsDouble(e.to);
                    open.add(new NodeRecord(e.to, f, gTent));
                }
            }
        }
        throw new RuntimeException("No path found");
    }
}
      
