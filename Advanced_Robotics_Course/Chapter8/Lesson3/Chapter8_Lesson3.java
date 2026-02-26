import java.util.*;

// Lightweight linear algebra can be provided by EJML or similar libraries.
class GraspGraph {
    static class Edge {
        int to;
        double cost;
        Edge(int t, double c) { to = t; cost = c; }
    }

    private final List<List<Edge>> adj;

    public GraspGraph(int n) {
        adj = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            adj.add(new ArrayList<>());
        }
    }

    public void addEdge(int from, int to, double cost) {
        adj.get(from).add(new Edge(to, cost));
    }

    public List<Integer> dijkstra(int start, int goal) {
        int n = adj.size();
        double[] dist = new double[n];
        int[] parent = new int[n];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(parent, -1);

        class Node {
            int v; double d;
            Node(int v, double d) { this.v = v; this.d = d; }
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(
            Comparator.comparingDouble(a -> a.d)
        );

        dist[start] = 0.0;
        pq.add(new Node(start, 0.0));

        while (!pq.isEmpty()) {
            Node cur = pq.poll();
            if (cur.d > dist[cur.v]) continue;
            if (cur.v == goal) break;
            for (Edge e : adj.get(cur.v)) {
                double nd = cur.d + e.cost;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    parent[e.to] = cur.v;
                    pq.add(new Node(e.to, nd));
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        if (Double.isInfinite(dist[goal])) return path;
        for (int v = goal; v != -1; v = parent[v]) {
            path.add(v);
        }
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args) {
        GraspGraph gg = new GraspGraph(4);
        gg.addEdge(0, 1, 1.0);
        gg.addEdge(1, 2, 0.8);
        gg.addEdge(2, 3, 0.9);
        gg.addEdge(0, 3, 3.5);

        List<Integer> path = gg.dijkstra(0, 3);
        System.out.println("Regrasp path: " + path);
    }
}
      
